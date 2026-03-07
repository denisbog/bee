//! Meshtastic AES-CTR Decryption
//! Standard decryption when PKI is not used
use aes::{Aes128, Aes256};
use alloc::vec::Vec;
use ctr::cipher::{KeyIvInit, StreamCipher};

type Aes128Ctr = ctr::Ctr128BE<Aes128>;
type Aes256Ctr = ctr::Ctr128BE<Aes256>;
// Key sizes
const AES_KEY_SIZE_128: usize = 16;
const AES_KEY_SIZE_256: usize = 32;
const NONCE_SIZE: usize = 16;
pub const DEFAULT_KEY: [u8; 32] = [
    0x68, 0x4a, 0xf7, 0x58, 0xe2, 0x67, 0xb0, 0xcc, 0xe6, 0x31, 0xdb, 0xce, 0xb5, 0x8c, 0x81, 0x43,
    0x0c, 0x79, 0x83, 0xb1, 0xe3, 0x42, 0x0e, 0x17, 0x90, 0x7c, 0x60, 0xb0, 0x47, 0x2c, 0x28,
    0xd0, // 0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
];
/// Decrypts a Meshtastic packet payload using AES-CTR
///
/// # Arguments
/// * `key` - Pre-shared key (16 bytes for AES-128, 32 bytes for AES-256)
/// * `from_node` - Source node number (uint32)
/// * `packet_id` - Unique packet identifier (uint64)
/// * `ciphertext` - Encrypted payload bytes
///
/// # Returns
/// * `Ok(Vec<u8>)` - Decrypted plaintext
/// * `Err(&'static str)` - Error message
pub fn decrypt_aes_ctr(
    key: &[u8],
    from_node: u32,
    packet_id: u64,
    ciphertext: &[u8],
) -> Result<Vec<u8>, &'static str> {
    // Validate key size
    if key.len() != AES_KEY_SIZE_128 && key.len() != AES_KEY_SIZE_256 {
        return Err("Invalid key size: must be 16 or 32 bytes");
    }

    if ciphertext.is_empty() {
        return Err("Ciphertext is empty");
    }
    // Build nonce (same as encryption)
    let mut nonce = [0u8; NONCE_SIZE];

    // Bytes 0-7: packet_id (little-endian)
    nonce[0..8].copy_from_slice(&packet_id.to_le_bytes());

    // Bytes 8-11: from_node (little-endian)
    nonce[8..12].copy_from_slice(&from_node.to_le_bytes());

    // Bytes 12-15: zeros (already initialized)
    // Create cipher based on key size
    let mut plaintext = ciphertext.to_vec();

    if key.len() == AES_KEY_SIZE_128 {
        // AES-128-CTR
        let mut cipher = Aes128Ctr::new(key.into(), (&nonce).into());
        cipher.apply_keystream(&mut plaintext);
    } else {
        // AES-256-CTR
        let mut cipher = Aes256Ctr::new(key.into(), (&nonce).into());
        cipher.apply_keystream(&mut plaintext);
    }
    Ok(plaintext)
}

/// Parse the MeshPacket to extract fields needed for decryption
#[derive(Debug)]
pub struct MeshPacketInfo {
    pub from_node: u32,
    pub packet_id: u64,
    pub channel: u8,
    pub encrypted_payload: Vec<u8>,
}
/// Extract decryption parameters from raw packet bytes
///
/// The packet header (16 bytes) is NOT encrypted:
/// - Bytes 0-3: to (uint32)
/// - Bytes 4-7: from (uint32)  
/// - Bytes 8-11: id (uint32 in header, but upper bits may differ)
/// - Byte 12: flags
/// - Byte 13: channel
/// - Byte 14: next_hop
/// - Byte 15: relay_node
///
/// Note: Full packet_id is in the encrypted payload area
pub fn parse_packet_header(data: &[u8]) -> Result<MeshPacketInfo, &'static str> {
    if data.len() < 16 {
        return Err("Packet too short: need at least 16 bytes for header");
    }

    let _to = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    let from = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
    let id = u32::from_le_bytes([data[8], data[9], data[10], data[11]]);
    let _flags = data[12];
    let channel = data[13];
    let _next_hop = data[14];
    let _relay_node = data[15];

    // The encrypted payload starts at byte 16
    let encrypted_payload = data[16..].to_vec();

    Ok(MeshPacketInfo {
        from_node: from,
        packet_id: id as u64, // Note: Full ID may need to be reconstructed
        channel,
        encrypted_payload,
    })
}
