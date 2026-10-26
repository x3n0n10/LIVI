pub const APPLE_VID: u16 = 0x05ac;
pub const CP_CONFIG: u8 = 6;
/// Error prefix when a device refuses to expose its CarPlay configurations.
pub const NO_CARPLAY_CONFIGS: &str = "vendor request 0x52";

pub mod ntb;

mod pipe;
pub use pipe::{MuxReader, MuxWriter, PhoneInfo};

mod backend;
pub use backend::{ensure_carplay_config, find_iphones, open_pipes, reset, restore_default_config};

pub const EP_OUT: u8 = 0x04;
pub const EP_IN: u8 = 0x85;

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "linux")]
pub use linux::{IPhoneDev, open_by_address, request_role_switch, restore_all_default_config};

#[cfg(target_os = "linux")]
mod usb_pipe;

mod mux;
pub use mux::{LOCKDOWN_PORT, MuxHost, MuxTcpConn};

mod device;
pub use device::{MuxDevice, MuxRegistry, socket_path};

#[cfg(target_os = "linux")]
mod ncm;
#[cfg(target_os = "linux")]
pub use ncm::{NcmBridge, drop_name_bound_profiles, link_local_profile};

mod async_stream;
pub use async_stream::AsyncMuxStream;
