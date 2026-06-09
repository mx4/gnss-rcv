use core::sync::atomic::Ordering;
use rustfft::num_complex::Complex64;
use std::collections::VecDeque;
use std::io::Read;
use std::io::Write;
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::thread;
use std::thread::JoinHandle;
use std::time::Instant;

use crate::code::Signal;
use crate::receiver::IQReader;

pub struct RtlSdrTcp {
    iq_deque: Arc<Mutex<VecDeque<Vec<Complex64>>>>,
    num_samples_total: Arc<Mutex<usize>>,
    num_samples: Arc<Mutex<usize>>,
    num_sleep: u64,
    read_th: Option<JoinHandle<()>>,
    ts: Instant,
}

impl Drop for RtlSdrTcp {
    fn drop(&mut self) {
        let tot = *self.num_samples_total.lock().unwrap();
        log::warn!(
            "num_samples={}/{} sleep={} -- {:.2} sec. rate={:.1}/sec",
            self.num_samples.lock().unwrap(),
            tot,
            self.num_sleep,
            self.ts.elapsed().as_secs_f64(),
            tot as f64 / self.ts.elapsed().as_secs_f64()
        );
    }
}

impl IQReader for RtlSdrTcp {
    fn get_iq_data(
        &mut self,
        _off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>> {
        loop {
            if *self.num_samples.lock().unwrap() >= num_samples {
                break;
            }
            thread::sleep(std::time::Duration::from_millis(1));
            self.num_sleep += 1;
        }
        let mut vec = vec![];
        let mut iq_deq = self.iq_deque.lock().unwrap();

        let v_front = iq_deq.front_mut().unwrap();
        let n = usize::min(num_samples, v_front.len());
        for v in v_front.iter().take(n) {
            vec.push(*v);
        }
        let _ = v_front.drain(0..n);
        if v_front.is_empty() {
            let _ = iq_deq.pop_front();
        }

        if n < num_samples {
            let v_front = iq_deq.front_mut().unwrap();
            for i in n..num_samples {
                vec.push(v_front[i - n]);
            }
            let _ = v_front.drain(0..num_samples - n);
        }

        *self.num_samples.lock().unwrap() -= num_samples;

        Ok(vec)
    }
}

fn rtl_sdr_send_cmd(socket: &mut TcpStream, cmd: u8, param: u32) -> std::io::Result<()> {
    let res = socket.write(&cmd.to_be_bytes())?;
    assert_eq!(res, 1);
    let res = socket.write(&param.to_be_bytes())?;
    assert_eq!(res, 4);
    Ok(())
}

impl RtlSdrTcp {
    pub fn new(
        hostname: &str,
        exit_req: Arc<AtomicBool>,
        sig: Signal,
        fs: f64,
    ) -> std::io::Result<RtlSdrTcp> {
        let mut socket = TcpStream::connect(hostname)?;

        let mut m = RtlSdrTcp {
            iq_deque: Arc::new(Mutex::new(VecDeque::new())),
            num_samples_total: Arc::new(Mutex::new(0)),
            num_samples: Arc::new(Mutex::new(0)),
            num_sleep: 0,
            read_th: None,
            ts: Instant::now(),
        };

        let iq_deq = m.iq_deque.clone();
        let num_samples_total = m.num_samples_total.clone();
        let num_samples = m.num_samples.clone();

        // set bias-t
        rtl_sdr_send_cmd(&mut socket, 0xe, 1)?;
        // set automatic gain control
        rtl_sdr_send_cmd(&mut socket, 0x8, 1)?;
        // set center frequency
        rtl_sdr_send_cmd(&mut socket, 0x1, sig.carrier_hz() as u32)?;
        // set sample rate
        rtl_sdr_send_cmd(&mut socket, 0x2, fs as u32)?;

        // set tuner gain
        //rtl_sdr_send_cmd(&mut socket, 0x4, 480)?;

        let th = thread::spawn(move || {
            loop {
                let mut data = [0u8; 2036 * 2];
                let mut v = vec![Complex64::default(); data.len()];
                let res = socket.read_exact(&mut data);
                if res.is_err() {
                    log::warn!("Failed to read from rtl-sdr");
                    exit_req.store(true, Ordering::SeqCst);
                    break;
                }

                for i in 0..data.len() / 2 {
                    let re = (data[2 * i] as f64 - 127.3) / 128.0;
                    let im = (data[2 * i + 1] as f64 - 127.3) / 128.0;

                    v[i] = Complex64 { re, im };
                }

                let n = v.len();
                iq_deq.lock().unwrap().push_back(v);
                *num_samples.lock().unwrap() += n;
                *num_samples_total.lock().unwrap() += n;
            }
        });
        m.read_th = Some(th);

        Ok(m)
    }
}
