#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_camel_case_types)]

use std::convert::TryFrom;
use std::io::prelude::*;
use std::io::BufReader;
use std::str;
use std::str::FromStr;
use std::thread::sleep;
use std::time::Duration;
use std::io::stdin;
use std::fs::File;
use std::path::Path;

use num_enum::TryFromPrimitive;

use anyhow::{anyhow, Result};

use nom::{
    branch::alt,
    bytes::complete::{escaped, tag, take_till, take_until, take_while},
    character::complete::{alphanumeric1 as alphanumeric, char, digit1, one_of},
    combinator::{cut, map, opt, rest, value},
    error::{context, convert_error},
    multi::separated_list,
    number::complete::{be_u64, double},
    sequence::{delimited, preceded, separated_pair, terminated},
    Err, IResult,
};

#[derive(Debug, PartialEq, Clone)]
pub enum ResponseTag {
    OK,
    ATE0,
    COPS,
    CEREG,
    CREG,
    UNKNOWN,
}

impl FromStr for ResponseTag {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        Ok(match s {
            "OK" => ResponseTag::OK,
            "ATE0" => ResponseTag::ATE0,
            "COPS" => ResponseTag::COPS,
            "CEREG" => ResponseTag::CEREG,
            "CREG" => ResponseTag::CREG,
            _ => ResponseTag::UNKNOWN,
        })
    }
}

#[derive(Debug, Eq, PartialEq, Clone)]
pub enum ATValue {
    Str(String),
    Num(u64),
}

impl ATValue {
    fn as_str(&self) -> Result<&str> {
        if let ATValue::Str(s) = self {
            Ok(s)
        } else {
            Err(anyhow!("Not a string"))
        }
    }

    fn as_num(&self) -> Result<u64> {
        if let ATValue::Num(n) = self {
            Ok(*n)
        } else {
            Err(anyhow!("Not a number"))
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone, TryFromPrimitive)]
#[repr(u64)]
enum CopsMode {
    Automatic = 0,
    ManualSelection = 1,
    ManualDeregister = 2,
    FormatOnly = 3,
    ManualThenAutomatic = 4,
}

#[derive(Debug, Eq, PartialEq, Clone, TryFromPrimitive)]
#[repr(u64)]
enum CopsFormat {
    Long = 0,
    Short = 1,
    Numeric = 2,
}

#[derive(Debug, Eq, PartialEq, Clone, TryFromPrimitive)]
#[repr(u64)]
enum AccessTech {
    GSM = 0,
    UTRAN = 2,
    GSM_EGPRS = 3,
    UTRAN_HSDPA = 4,
    UTRAN_HSUPA = 5,
    UTRAN_HSDPA_HSUPA = 6,
    E_UTRAN = 7,
    CDMA = 100,
}

#[derive(Debug, Eq, PartialEq, Clone, TryFromPrimitive)]
#[repr(u64)]
enum RegistrationN {
    DisableRegistration = 0,
    EnableRegistration = 1,
    EnableRegistrationLocation = 2,
}

#[derive(Debug, Eq, PartialEq, Clone, TryFromPrimitive)]
#[repr(u64)]
enum RegistrationStatus {
    NotRegistered = 0,
    RegisteredHome = 1,
    Searching = 2,
    Denied = 3,
    Unknown = 4,
    RegisteredRoaming = 5,
}

#[derive(Debug, Eq, PartialEq, Clone)]
struct CopsResponse {
    mode: CopsMode,
    format: Option<CopsFormat>,
    operator: Option<String>,
    access_tech: Option<AccessTech>,
}

impl TryFrom<Vec<ATValue>> for CopsResponse {
    type Error = anyhow::Error;

    fn try_from(values: Vec<ATValue>) -> Result<Self> {
        let (mode, format, operator, access_tech) = match &values[..] {
            [m, f, o, a] => (m, Some(f), Some(o), Some(a)),
            [m, f, o] => (m, Some(f), Some(o), None),
            [m, f] => (m, Some(f), None, None),
            [m] => (m, None, None, None),
            _ => {
                return Err(anyhow!(
                    "Incorrect number of AT values for COPS response - {}",
                    values.len()
                ))
            }
        };

        let mode = CopsMode::try_from(mode.as_num()?)?;

        let format = if let Some(f) = format {
            Some(CopsFormat::try_from(f.as_num()?)?)
        } else {
            None
        };

        let operator = if let Some(o) = operator {
            Some(o.as_str()?.into())
        } else {
            None
        };

        let access_tech = if let Some(a) = access_tech {
            Some(AccessTech::try_from(a.as_num()?)?)
        } else {
            None
        };

        Ok(CopsResponse {
            mode,
            format,
            operator,
            access_tech,
        })
    }
}

#[derive(Debug, Eq, PartialEq, Clone)]
struct CregResponse {
    n: RegistrationN,
    status: RegistrationStatus,
}

impl TryFrom<Vec<ATValue>> for CregResponse {
    type Error = anyhow::Error;

    fn try_from(values: Vec<ATValue>) -> Result<Self> {
        if let [n, status] = &values[..] {
            let n = RegistrationN::try_from(n.as_num()?)?;
            let status = RegistrationStatus::try_from(status.as_num()?)?;
            Ok(CregResponse { n, status })
        } else {
            Err(anyhow!(
                "Incorrect number of AT values for CREG response - {}",
                values.len()
            ))
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone)]
struct CeregResponse {
    n: RegistrationN,
    status: RegistrationStatus,
}

impl TryFrom<Vec<ATValue>> for CeregResponse {
    type Error = anyhow::Error;

    fn try_from(values: Vec<ATValue>) -> Result<Self> {
        if let [n, status] = &values[..] {
            let n = RegistrationN::try_from(n.as_num()?)?;
            let status = RegistrationStatus::try_from(status.as_num()?)?;
            Ok(CeregResponse { n, status })
        } else {
            Err(anyhow!(
                "Incorrect number of AT values for CEREG response - {}",
                values.len()
            ))
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone)]
enum Response {
    Cops(CopsResponse),
    Creg(CregResponse),
    Cereg(CeregResponse),
}

fn main() -> Result<()> {
    let mut port = serialport::new("/dev/ttyUSB2", libc::B115200)
        .timeout(Duration::from_millis(5000))
        .open_native()?;

    echo_off(&mut port)?;

    cmd(&mut port, "AT")?;

    cmd(&mut port, "AT+CFUN=0")?;

    sleep(Duration::from_secs(3));

    cmd(&mut port, "AT+COPS?")?;
    cmd(&mut port, "AT+CEREG?")?;
    cmd(&mut port, "AT+CREG?")?;

    create_lock()?;

    println!("Enter SIM and delete lock file");

    let mut locked = true;
    while locked {
        locked = lock_exists();
        sleep(Duration::from_secs(1));
    }

    cmd(&mut port, "AT+QCFG=\"nwscanmode\",0")?;

    cmd(&mut port, "AT+CFUN=1")?;

    loop {
        cmd(&mut port, "AT+COPS?")?;
        cmd(&mut port, "AT+CEREG?")?;
        cmd(&mut port, "AT+CREG?")?;

        sleep(Duration::from_secs(10));
    }
}

fn create_lock() -> Result<()> {
    File::create("at.lock")?;
    Ok(())
}

fn lock_exists() -> bool {
    Path::new("at.lock").exists()
}

fn echo_off(port: &mut serialport::TTYPort) -> Result<()> {
    write(port, "ATE0")?;
    let r = read(port)?;
    if r == ResponseTag::ATE0 {
        read(port)?;
    }
    Ok(())
}

fn cmd(port: &mut serialport::TTYPort, at_cmd: &str) -> Result<ResponseTag> {
    println!(">>> {}", at_cmd);
    write(port, at_cmd)?;
    read(port)
}

fn write(port: &mut serialport::TTYPort, at_cmd: &str) -> Result<()> {
    let with_new_line = format!("{}\r\n", at_cmd);
    port.write_all(with_new_line.as_bytes())?;
    port.flush()?;
    Ok(())
}

fn read(port: &mut serialport::TTYPort) -> Result<ResponseTag> {
    let res = read_line(port)?;
    println!("<<< {}", res);
    let (_, (tag, values)) = response(&res).map_err(|_| anyhow!("Parsing failed"))?;
    let tag = ResponseTag::from_str(tag)?;
    match tag {
        ResponseTag::COPS => {
            println!("{:?}", CopsResponse::try_from(values)?);
        }
        ResponseTag::CEREG => {
            println!("{:?}", CeregResponse::try_from(values)?);
        }
        ResponseTag::CREG => {
            println!("{:?}", CregResponse::try_from(values)?);
        }
        _ => {}
    }
    Ok(tag)
}

fn read_line(port: &mut serialport::TTYPort) -> Result<String> {
    let mut reader = BufReader::new(port);

    loop {
        let mut line = String::new();
        reader.read_line(&mut line)?;
        let line = line.trim_end().to_string();

        if !line.is_empty() {
            return Ok(line);
        }
    }
}

fn ok(input: &str) -> IResult<&str, (&str, Vec<ATValue>)> {
    from_tag("OK", input)
}

fn ate0(input: &str) -> IResult<&str, (&str, Vec<ATValue>)> {
    from_tag("ATE0", input)
}

fn from_tag<'a>(name: &'static str, input: &'a str) -> IResult<&'a str, (&'a str, Vec<ATValue>)> {
    map(tag(name), |s: &str| (s, vec![]))(input)
}

fn string(input: &str) -> IResult<&str, String> {
    map(
        preceded(char('"'), terminated(take_until("\""), char('"'))),
        |s: &str| s.to_string(),
    )(input)
}

fn number(input: &str) -> IResult<&str, u64> {
    map(digit1, |s: &str| s.parse::<u64>().unwrap())(input)
}

fn at_value(input: &str) -> IResult<&str, ATValue> {
    alt((map(string, ATValue::Str), map(number, ATValue::Num)))(input)
}

fn at_value_list(input: &str) -> IResult<&str, Vec<ATValue>> {
    separated_list(char(','), at_value)(input)
}

fn command(input: &str) -> IResult<&str, (&str, Vec<ATValue>)> {
    separated_pair(
        preceded(char('+'), take_until(":")),
        tag(": "),
        at_value_list,
    )(input)
}

fn response(input: &str) -> IResult<&str, (&str, Vec<ATValue>)> {
    alt((ok, ate0, command))(input)
}
