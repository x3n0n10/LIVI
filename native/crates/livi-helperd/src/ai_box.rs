//! Wired CarPlay for "AI boxes": Android boxes that pose as an iPhone on the
//! cable but refuse the CarPlay configuration request (0x52). They answer the
//! older role switch (0x51) instead and then act as the USB host, so LIVI has
//! to show up as the accessory: a USB gadget with the iAP2 interface and NCM.
//!
//! The box sends the role switch on a host port, and the gadget lives on the
//! board's device port (the Pi 4's USB-C with `dtoverlay=dwc2,dr_mode=peripheral`).
//! shortcut: with one cable the user moves it from the host to the device
//! port within a few seconds; an OTG port that switches roles itself would
//! make that automatic.

use std::fs;
use std::io::{Read, Write};
use std::os::unix::fs::symlink;
use std::path::Path;
use std::process::Command;
use std::sync::{Arc, OnceLock};

use iap2_link::LinkConfig;
use tokio::sync::{Notify, mpsc};

use crate::wired::{LocalNcm, WiredCtx, run_wired_session, short};

const UDC_DIR: &str = "/sys/class/udc";
const GADGET: &str = "/sys/kernel/config/usb_gadget/livi_cp";
const FFS_DIR: &str = "/dev/ffs-livi-iap2";
const FFS_NAME: &str = "livi_iap2";
const NCM: &str = "ncm.usb0";
const FFS: &str = "ffs.livi_iap2";
// a freshly powered box needs ~20 s before it honours the role switch
const BOOT_GRACE: std::time::Duration = std::time::Duration::from_secs(25);

const LINK: LinkConfig = LinkConfig {
    max_outgoing: 4,
    max_outgoing_delta: 0,
    ack_timeout_ms: 500,
    zero_ack: false,
    control_version: 2,
};

// functionfs event types, linux/usb/functionfs.h
const EV_UNBIND: u8 = 1;
const EV_ENABLE: u8 = 2;
const EV_DISABLE: u8 = 3;
const EV_SETUP: u8 = 4;

static GADGET_UP: OnceLock<mpsc::UnboundedSender<(String, WiredCtx)>> = OnceLock::new();

/// True when the board has a USB device controller to put the gadget on.
pub fn available() -> bool {
    udc().is_some()
}

/// Brings the gadget up (once), then asks the box for the role switch.
pub fn switch(serial: &str, ctx: WiredCtx) {
    let boxes = match GADGET_UP.get() {
        Some(tx) => tx.clone(),
        None => match start() {
            Ok(tx) => GADGET_UP.get_or_init(|| tx).clone(),
            Err(e) => {
                eprintln!("[aibox] no accessory gadget: {e}");
                return;
            }
        },
    };
    let _ = boxes.send((serial.to_string(), ctx));
    let serial = serial.to_string();
    println!(
        "[aibox] {}: AI box found, role switch in {}s once it has booted",
        short(&serial),
        BOOT_GRACE.as_secs()
    );
    tokio::spawn(async move {
        // shortcut: a fixed wait, since the box only takes the host role once
        // its Android side is up and treats an early 0x51 as a plain reset;
        // replace with a readiness probe if one turns up.
        tokio::time::sleep(BOOT_GRACE).await;
        let target = serial.clone();
        match tokio::task::spawn_blocking(move || iap2_usbmux::request_role_switch(&target)).await {
            Ok(Ok(())) => println!(
                "[aibox] {}: asked for the USB role switch — move the cable to the device port now",
                short(&serial)
            ),
            Ok(Err(e)) => eprintln!("[aibox] {}: {e}", short(&serial)),
            Err(e) => eprintln!("[aibox] {}: role switch task: {e}", short(&serial)),
        }
    });
}

fn udc() -> Option<String> {
    fs::read_dir(UDC_DIR)
        .ok()?
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .next()
}

fn start() -> Result<mpsc::UnboundedSender<(String, WiredCtx)>, String> {
    let ep0 = build_gadget()?;
    let ifname = fs::read_to_string(format!("{GADGET}/functions/{NCM}/ifname"))
        .map(|s| s.trim().to_string())
        .map_err(|e| format!("ncm ifname: {e}"))?;
    println!("[aibox] accessory gadget up, NCM on {ifname}");

    let (events_tx, events) = mpsc::unbounded_channel();
    std::thread::spawn(move || read_events(ep0, events_tx));
    let (boxes_tx, boxes) = mpsc::unbounded_channel();
    tokio::spawn(run(events, boxes, ifname));
    Ok(boxes_tx)
}

/// Pairs each ENABLE (the box took the host role and picked our config) with
/// the last box we role-switched, and runs a wired session until DISABLE.
async fn run(
    mut events: mpsc::UnboundedReceiver<u8>,
    mut boxes: mpsc::UnboundedReceiver<(String, WiredCtx)>,
    ifname: String,
) {
    let mut last: Option<(String, WiredCtx)> = None;
    let mut cancel: Option<Arc<Notify>> = None;
    loop {
        tokio::select! {
            b = boxes.recv() => match b {
                Some(b) => last = Some(b),
                None => return,
            },
            ev = events.recv() => match ev {
                Some(EV_ENABLE) => {
                    let Some((serial, ctx)) = last.clone() else {
                        println!("[aibox] a host enabled the gadget, but no box asked for it");
                        continue;
                    };
                    println!("[aibox] {}: box is host now, starting iAP2", short(&serial));
                    let stream = match open_endpoints() {
                        Ok(s) => s,
                        Err(e) => {
                            eprintln!("[aibox] {e}");
                            continue;
                        }
                    };
                    let name = ifname.clone();
                    let _ = tokio::task::spawn_blocking(move || iap2_usbmux::link_local_profile(&name)).await;
                    let c = Arc::new(Notify::new());
                    cancel = Some(c.clone());
                    let ncm = LocalNcm::System(Some(ifname.clone()));
                    tokio::spawn(run_wired_session(serial, stream, ncm, LINK, ctx, c));
                }
                Some(EV_DISABLE | EV_UNBIND) => {
                    if let Some(c) = cancel.take() {
                        println!("[aibox] box let go of the gadget");
                        c.notify_one();
                    }
                }
                Some(_) => {}
                None => return,
            },
        }
    }
}

fn open_endpoints()
-> Result<impl tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin + Send, String> {
    // ep1 is the bulk IN endpoint (to the box), ep2 the bulk OUT one, in descriptor order.
    let open = |n: u32, write: bool| {
        fs::OpenOptions::new()
            .read(!write)
            .write(write)
            .open(format!("{FFS_DIR}/ep{n}"))
            .map(tokio::fs::File::from_std)
            .map_err(|e| format!("open ep{n}: {e}"))
    };
    Ok(tokio::io::join(open(2, false)?, open(1, true)?))
}

fn read_events(mut ep0: fs::File, tx: mpsc::UnboundedSender<u8>) {
    let mut ev = [0u8; 12];
    loop {
        if let Err(e) = ep0.read_exact(&mut ev) {
            eprintln!("[aibox] gadget ep0 ended: {e}");
            return;
        }
        let kind = ev[8];
        if kind == EV_SETUP {
            // We answer no control requests: reading or writing against the
            // request's direction makes the kernel stall it.
            let device_to_host = ev[0] & 0x80 != 0;
            let _ = if device_to_host {
                ep0.read(&mut [0u8; 0])
            } else {
                ep0.write(&[])
            };
            eprintln!("[aibox] stalled control request {:02x?}", &ev[..8]);
            continue;
        }
        if tx.send(kind).is_err() {
            return;
        }
    }
}

fn build_gadget() -> Result<fs::File, String> {
    for m in ["libcomposite", "usb_f_fs", "usb_f_ncm"] {
        let _ = Command::new("modprobe").arg(m).status();
    }
    teardown();
    let udc = udc().ok_or("no USB device controller")?;
    let w = |rel: &str, val: &str| {
        let p = format!("{GADGET}/{rel}");
        fs::write(&p, val).map_err(|e| format!("{p}: {e}"))
    };
    let mk = |rel: &str| {
        let p = format!("{GADGET}/{rel}");
        fs::create_dir_all(&p).map_err(|e| format!("{p}: {e}"))
    };
    mk("")?;
    w("idVendor", "0x1d6b")?;
    w("idProduct", "0x0104")?;
    w("bcdUSB", "0x0200")?;
    w("bcdDevice", "0x0100")?;
    mk("strings/0x409")?;
    w("strings/0x409/manufacturer", "LIVI")?;
    w("strings/0x409/product", "LIVI CarPlay")?;
    w("strings/0x409/serialnumber", "0001")?;
    mk("configs/c.1/strings/0x409")?;
    w("configs/c.1/strings/0x409/configuration", "CarPlay")?;
    w("configs/c.1/MaxPower", "500")?;
    mk(&format!("functions/{FFS}"))?;
    mk(&format!("functions/{NCM}"))?;
    // fixed addresses keep one NetworkManager profile across restarts
    w(&format!("functions/{NCM}/dev_addr"), "02:4c:49:56:49:01")?;
    w(&format!("functions/{NCM}/host_addr"), "02:4c:49:56:49:02")?;
    for f in [FFS, NCM] {
        symlink(
            format!("{GADGET}/functions/{f}"),
            format!("{GADGET}/configs/c.1/{f}"),
        )
        .map_err(|e| format!("link {f}: {e}"))?;
    }

    fs::create_dir_all(FFS_DIR).map_err(|e| format!("{FFS_DIR}: {e}"))?;
    let ok = Command::new("mount")
        .args(["-t", "functionfs", FFS_NAME, FFS_DIR])
        .status()
        .is_ok_and(|s| s.success());
    if !ok {
        return Err(format!("mount functionfs on {FFS_DIR}"));
    }
    let mut ep0 = fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(format!("{FFS_DIR}/ep0"))
        .map_err(|e| format!("open ep0: {e}"))?;
    ep0.write_all(&descriptors())
        .map_err(|e| format!("ffs descriptors: {e}"))?;
    ep0.write_all(&strings())
        .map_err(|e| format!("ffs strings: {e}"))?;
    w("UDC", &udc)?;
    Ok(ep0)
}

fn teardown() {
    let g = Path::new(GADGET);
    if !g.exists() {
        return;
    }
    let _ = fs::write(g.join("UDC"), "");
    for f in [FFS, NCM] {
        let _ = fs::remove_file(g.join("configs/c.1").join(f));
    }
    for d in [
        "configs/c.1/strings/0x409",
        "configs/c.1",
        &format!("functions/{FFS}"),
        &format!("functions/{NCM}"),
        "strings/0x409",
        "",
    ] {
        let _ = fs::remove_dir(g.join(d));
    }
    let _ = Command::new("umount").arg(FFS_DIR).status();
}

/// The iAP2 interface CarPlay accessories expose: vendor class 0xff,
/// subclass 0xf0, two bulk endpoints.
fn descriptors() -> Vec<u8> {
    fn speed(max_packet: u16) -> Vec<u8> {
        let [lo, hi] = max_packet.to_le_bytes();
        let mut d = vec![9, 4, 0, 0, 2, 0xff, 0xf0, 0x00, 1];
        d.extend([7, 5, 0x81, 2, lo, hi, 0]);
        d.extend([7, 5, 0x02, 2, lo, hi, 0]);
        d
    }
    let (fs, hs) = (speed(64), speed(512));
    let mut out = Vec::new();
    for v in [3u32, 0, 1 | 2, 3, 3] {
        // magic v2, length (patched below), flags fs|hs, fs count, hs count
        out.extend(v.to_le_bytes());
    }
    out.extend(fs);
    out.extend(hs);
    let len = (out.len() as u32).to_le_bytes();
    out[4..8].copy_from_slice(&len);
    out
}

fn strings() -> Vec<u8> {
    let mut out = Vec::new();
    for v in [2u32, 0, 1, 1] {
        // magic, length (patched below), string count, language count
        out.extend(v.to_le_bytes());
    }
    out.extend(0x0409u16.to_le_bytes());
    out.extend(b"iAP Interface\0");
    let len = (out.len() as u32).to_le_bytes();
    out[4..8].copy_from_slice(&len);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_blob_matches_the_functionfs_v2_layout() {
        let d = descriptors();
        assert_eq!(u32::from_le_bytes(d[0..4].try_into().unwrap()), 3);
        assert_eq!(
            u32::from_le_bytes(d[4..8].try_into().unwrap()) as usize,
            d.len()
        );
        assert_eq!(d.len(), 20 + 2 * (9 + 7 + 7));
        // high-speed bulk endpoints carry 512-byte packets
        assert_eq!(&d[20 + 23 + 9 + 4..20 + 23 + 9 + 6], &[0x00, 0x02]);
    }

    #[test]
    fn string_blob_holds_one_english_string() {
        let s = strings();
        assert_eq!(
            u32::from_le_bytes(s[4..8].try_into().unwrap()) as usize,
            s.len()
        );
        assert_eq!(&s[16..18], &[0x09, 0x04]);
        assert!(s.ends_with(b"iAP Interface\0"));
    }
}
