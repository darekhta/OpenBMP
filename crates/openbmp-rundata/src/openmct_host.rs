//! Feature-gated host example support for Open MCT replay.
//!
//! This module is intentionally outside the deterministic run-data core. It
//! serves already-generated Open MCT catalog JSON and replay frames over a
//! minimal loopback HTTP/WebSocket surface for host-side smoke testing.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, ToSocketAddrs};

use thiserror::Error;

use crate::openmct::OpenMctReplayFrame;

const MAX_REQUEST_BYTES: usize = 8192;
const WEBSOCKET_GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/// Errors returned by the Open MCT host example helpers.
#[derive(Debug, Error)]
pub enum OpenMctHostError {
    /// I/O failed while accepting, reading, or writing a host connection.
    #[error("Open MCT host I/O error: {source}")]
    Io {
        /// Source I/O error.
        #[from]
        source: std::io::Error,
    },
    /// Replay frame serialization failed.
    #[error("Open MCT host JSON error: {source}")]
    Json {
        /// Source JSON error.
        #[from]
        source: serde_json::Error,
    },
    /// The HTTP/WebSocket request was malformed or unsupported.
    #[error("Open MCT host invalid request: {reason}")]
    InvalidRequest {
        /// Human-readable reason.
        reason: String,
    },
}

/// Serve one accepted HTTP/WebSocket request.
///
/// `GET /catalog.json` returns the supplied catalog JSON. `GET /replay` must
/// request a WebSocket upgrade and then receives one text frame per replay
/// frame followed by a close frame. `GET /` returns a small host page.
///
/// # Errors
///
/// Returns [`OpenMctHostError`] when the stream cannot be read or written, the
/// request is malformed, or a replay frame cannot be serialized.
pub fn serve_openmct_stream<S: Read + Write>(
    mut stream: S,
    catalog_json: &[u8],
    replay_frames: &[OpenMctReplayFrame],
) -> Result<(), OpenMctHostError> {
    let request = read_http_request(&mut stream)?;
    let response = openmct_http_response(&request, catalog_json, replay_frames)?;
    stream.write_all(&response)?;
    Ok(())
}

/// Serve a bounded number of loopback Open MCT host requests.
///
/// # Errors
///
/// Returns [`OpenMctHostError`] when accepting or serving a connection fails.
pub fn serve_openmct_listener(
    listener: TcpListener,
    catalog_json: &[u8],
    replay_frames: &[OpenMctReplayFrame],
    max_connections: usize,
) -> Result<(), OpenMctHostError> {
    for _ in 0..max_connections {
        let (stream, _) = listener.accept()?;
        serve_openmct_stream(stream, catalog_json, replay_frames)?;
    }
    Ok(())
}

/// Bind a TCP listener for the Open MCT host example.
///
/// # Errors
///
/// Returns [`OpenMctHostError::Io`] when the address cannot be bound.
pub fn bind_openmct_listener<A: ToSocketAddrs>(
    address: A,
) -> Result<(TcpListener, SocketAddr), OpenMctHostError> {
    let listener = TcpListener::bind(address)?;
    let address = listener.local_addr()?;
    Ok((listener, address))
}

fn openmct_http_response(
    request: &[u8],
    catalog_json: &[u8],
    replay_frames: &[OpenMctReplayFrame],
) -> Result<Vec<u8>, OpenMctHostError> {
    let parsed = parse_http_request(request)?;
    match parsed.path {
        "/" => Ok(http_response(
            "200 OK",
            "text/html; charset=utf-8",
            index_html().as_bytes(),
        )),
        "/catalog.json" => Ok(http_response("200 OK", "application/json", catalog_json)),
        "/replay" => websocket_replay_response(&parsed, replay_frames),
        _ => Ok(http_response(
            "404 Not Found",
            "text/plain; charset=utf-8",
            b"not found\n",
        )),
    }
}

fn read_http_request<S: Read>(stream: &mut S) -> Result<Vec<u8>, OpenMctHostError> {
    let mut request = Vec::new();
    let mut buffer = [0_u8; 1024];
    loop {
        let read = stream.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        request.extend_from_slice(&buffer[..read]);
        if request.windows(4).any(|window| window == b"\r\n\r\n") {
            return Ok(request);
        }
        if request.len() > MAX_REQUEST_BYTES {
            return Err(OpenMctHostError::InvalidRequest {
                reason: "request header exceeded limit".to_owned(),
            });
        }
    }
    Err(OpenMctHostError::InvalidRequest {
        reason: "request ended before headers completed".to_owned(),
    })
}

#[derive(Debug)]
struct ParsedRequest<'a> {
    path: &'a str,
    headers: Vec<(&'a str, &'a str)>,
}

fn parse_http_request(request: &[u8]) -> Result<ParsedRequest<'_>, OpenMctHostError> {
    let request = std::str::from_utf8(request).map_err(|_| OpenMctHostError::InvalidRequest {
        reason: "request is not UTF-8".to_owned(),
    })?;
    let mut lines = request.split("\r\n");
    let request_line = lines
        .next()
        .ok_or_else(|| invalid_request("missing request line"))?;
    let mut parts = request_line.split_ascii_whitespace();
    let method = parts
        .next()
        .ok_or_else(|| invalid_request("missing request method"))?;
    let path = parts
        .next()
        .ok_or_else(|| invalid_request("missing request path"))?;
    let version = parts
        .next()
        .ok_or_else(|| invalid_request("missing request version"))?;
    if method != "GET" {
        return Err(invalid_request("only GET requests are supported"));
    }
    if version != "HTTP/1.1" {
        return Err(invalid_request("only HTTP/1.1 requests are supported"));
    }
    let headers = lines
        .take_while(|line| !line.is_empty())
        .map(|line| {
            let Some((name, value)) = line.split_once(':') else {
                return Err(invalid_request("malformed header"));
            };
            Ok((name.trim(), value.trim()))
        })
        .collect::<Result<Vec<_>, OpenMctHostError>>()?;
    Ok(ParsedRequest { path, headers })
}

fn websocket_replay_response(
    request: &ParsedRequest<'_>,
    replay_frames: &[OpenMctReplayFrame],
) -> Result<Vec<u8>, OpenMctHostError> {
    if !header_contains(request, "connection", "upgrade")
        || !header_equals(request, "upgrade", "websocket")
    {
        return Err(invalid_request(
            "GET /replay requires a WebSocket upgrade request",
        ));
    }
    let Some(key) = header_value(request, "sec-websocket-key") else {
        return Err(invalid_request("missing Sec-WebSocket-Key header"));
    };
    let accept = websocket_accept_key(key);
    let mut response = Vec::new();
    response.extend_from_slice(b"HTTP/1.1 101 Switching Protocols\r\n");
    response.extend_from_slice(b"Upgrade: websocket\r\n");
    response.extend_from_slice(b"Connection: Upgrade\r\n");
    response.extend_from_slice(b"Sec-WebSocket-Accept: ");
    response.extend_from_slice(accept.as_bytes());
    response.extend_from_slice(b"\r\n\r\n");
    for frame in replay_frames {
        let payload = serde_json::to_vec(frame)?;
        write_websocket_text_frame(&mut response, &payload);
    }
    response.extend_from_slice(&[0x88, 0x00]);
    Ok(response)
}

fn http_response(status: &str, content_type: &str, body: &[u8]) -> Vec<u8> {
    let mut response = Vec::new();
    response.extend_from_slice(format!("HTTP/1.1 {status}\r\n").as_bytes());
    response.extend_from_slice(format!("Content-Type: {content_type}\r\n").as_bytes());
    response.extend_from_slice(format!("Content-Length: {}\r\n", body.len()).as_bytes());
    response.extend_from_slice(b"Connection: close\r\n\r\n");
    response.extend_from_slice(body);
    response
}

fn index_html() -> &'static str {
    "<!doctype html><meta charset=\"utf-8\"><title>OpenBMP Open MCT Replay</title><body><h1>OpenBMP Open MCT Replay</h1><p>Catalog: <code>/catalog.json</code></p><p>Replay WebSocket: <code>/replay</code></p></body>\n"
}

fn header_value<'a>(request: &'a ParsedRequest<'a>, name: &str) -> Option<&'a str> {
    request
        .headers
        .iter()
        .find(|(candidate, _)| candidate.eq_ignore_ascii_case(name))
        .map(|(_, value)| *value)
}

fn header_equals(request: &ParsedRequest<'_>, name: &str, expected: &str) -> bool {
    header_value(request, name).is_some_and(|value| value.eq_ignore_ascii_case(expected))
}

fn header_contains(request: &ParsedRequest<'_>, name: &str, expected: &str) -> bool {
    header_value(request, name).is_some_and(|value| {
        value
            .split(',')
            .any(|part| part.trim().eq_ignore_ascii_case(expected))
    })
}

fn websocket_accept_key(key: &str) -> String {
    let mut input = Vec::with_capacity(key.len() + WEBSOCKET_GUID.len());
    input.extend_from_slice(key.as_bytes());
    input.extend_from_slice(WEBSOCKET_GUID.as_bytes());
    base64_encode(&sha1_digest(&input))
}

fn write_websocket_text_frame(out: &mut Vec<u8>, payload: &[u8]) {
    out.push(0x81);
    match payload.len() {
        0..=125 => out.push(payload.len() as u8),
        126..=65_535 => {
            out.push(126);
            out.extend_from_slice(&(payload.len() as u16).to_be_bytes());
        }
        _ => {
            out.push(127);
            out.extend_from_slice(&(payload.len() as u64).to_be_bytes());
        }
    }
    out.extend_from_slice(payload);
}

fn invalid_request(reason: impl Into<String>) -> OpenMctHostError {
    OpenMctHostError::InvalidRequest {
        reason: reason.into(),
    }
}

fn base64_encode(bytes: &[u8]) -> String {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let first = chunk[0];
        let second = *chunk.get(1).unwrap_or(&0);
        let third = *chunk.get(2).unwrap_or(&0);
        out.push(ALPHABET[(first >> 2) as usize] as char);
        out.push(ALPHABET[(((first & 0x03) << 4) | (second >> 4)) as usize] as char);
        if chunk.len() > 1 {
            out.push(ALPHABET[(((second & 0x0f) << 2) | (third >> 6)) as usize] as char);
        } else {
            out.push('=');
        }
        if chunk.len() > 2 {
            out.push(ALPHABET[(third & 0x3f) as usize] as char);
        } else {
            out.push('=');
        }
    }
    out
}

fn sha1_digest(input: &[u8]) -> [u8; 20] {
    let mut hash = [
        0x6745_2301_u32,
        0xefcd_ab89_u32,
        0x98ba_dcfe_u32,
        0x1032_5476_u32,
        0xc3d2_e1f0_u32,
    ];
    let bit_len = (input.len() as u64) * 8;
    let mut message = input.to_vec();
    message.push(0x80);
    while message.len() % 64 != 56 {
        message.push(0);
    }
    message.extend_from_slice(&bit_len.to_be_bytes());

    for block in message.chunks_exact(64) {
        let mut words = [0_u32; 80];
        for (index, word) in words.iter_mut().take(16).enumerate() {
            let offset = index * 4;
            *word = u32::from_be_bytes([
                block[offset],
                block[offset + 1],
                block[offset + 2],
                block[offset + 3],
            ]);
        }
        for index in 16..80 {
            words[index] =
                (words[index - 3] ^ words[index - 8] ^ words[index - 14] ^ words[index - 16])
                    .rotate_left(1);
        }

        let mut a = hash[0];
        let mut b = hash[1];
        let mut c = hash[2];
        let mut d = hash[3];
        let mut e = hash[4];
        for (index, word) in words.iter().enumerate() {
            let (f, k) = match index {
                0..=19 => ((b & c) | ((!b) & d), 0x5a82_7999),
                20..=39 => (b ^ c ^ d, 0x6ed9_eba1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8f1b_bcdc),
                _ => (b ^ c ^ d, 0xca62_c1d6),
            };
            let temp = a
                .rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(*word);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = temp;
        }
        hash[0] = hash[0].wrapping_add(a);
        hash[1] = hash[1].wrapping_add(b);
        hash[2] = hash[2].wrapping_add(c);
        hash[3] = hash[3].wrapping_add(d);
        hash[4] = hash[4].wrapping_add(e);
    }

    let mut digest = [0_u8; 20];
    for (index, word) in hash.iter().enumerate() {
        digest[index * 4..index * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    digest
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::{TcpListener, TcpStream};
    use std::thread;

    fn frames() -> Vec<OpenMctReplayFrame> {
        vec![OpenMctReplayFrame {
            timestamp_s: 1.0,
            replay_time_s: 0.5,
            values: [("altitude_m".to_owned(), 42.0)].into(),
        }]
    }

    #[test]
    fn openmct_host_websocket_accept_matches_rfc_example() {
        assert_eq!(
            websocket_accept_key("dGhlIHNhbXBsZSBub25jZQ=="),
            "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="
        );
    }

    #[test]
    fn openmct_host_serves_catalog_http() {
        let response = openmct_http_response(
            b"GET /catalog.json HTTP/1.1\r\nHost: localhost\r\n\r\n",
            br#"{"schema_version":"openbmp.rundata.openmct.v1"}"#,
            &frames(),
        )
        .unwrap();
        let response = String::from_utf8(response).unwrap();
        assert!(response.starts_with("HTTP/1.1 200 OK\r\n"));
        assert!(response.contains("Content-Type: application/json\r\n"));
        assert!(response.ends_with(r#"{"schema_version":"openbmp.rundata.openmct.v1"}"#));
    }

    #[test]
    fn openmct_host_websocket_replay_smoke() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let handle = thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            serve_openmct_stream(stream, b"{}", &frames()).unwrap();
        });

        let mut stream = TcpStream::connect(address).unwrap();
        stream
            .write_all(
                b"GET /replay HTTP/1.1\r\n\
                  Host: localhost\r\n\
                  Upgrade: websocket\r\n\
                  Connection: Upgrade\r\n\
                  Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\
                  Sec-WebSocket-Version: 13\r\n\r\n",
            )
            .unwrap();
        let mut response = Vec::new();
        stream.read_to_end(&mut response).unwrap();
        handle.join().unwrap();

        let header_end = response
            .windows(4)
            .position(|window| window == b"\r\n\r\n")
            .expect("headers");
        let headers = String::from_utf8(response[..header_end].to_vec()).unwrap();
        assert!(headers.starts_with("HTTP/1.1 101 Switching Protocols\r\n"));
        assert!(headers.contains("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo="));
        assert!(
            response[header_end + 4..]
                .windows(10)
                .any(|window| window == b"altitude_m")
        );
    }
}
