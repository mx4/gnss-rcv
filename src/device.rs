use rustfft::num_complex::Complex64;
use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;

use crate::code::Signal;
use crate::receiver::IQReader;

pub struct RtlSdrDevice {
    controller: rtlsdr_mt::Controller,
    iq_deque: Arc<Mutex<VecDeque<Vec<Complex64>>>>,
    num_samples_total: Arc<Mutex<usize>>,
    num_samples: Arc<Mutex<usize>>,
    num_sleep: u64,
}

impl Drop for RtlSdrDevice {
    fn drop(&mut self) {
        log::warn!(
            "rtlsdr: stopping read. num_samples={}",
            self.num_samples.lock().unwrap()
        );
        log::warn!("rtlsdr: num_sleep={}", self.num_sleep);

        self.controller.cancel_async_read();
    }
}

impl IQReader for RtlSdrDevice {
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

impl RtlSdrDevice {
    #[allow(clippy::result_unit_err)]
    pub fn new(sig: Signal, fs: f64) -> Result<RtlSdrDevice, ()> {
        let devices = rtlsdr_mt::devices();

        for dev in devices {
            log::warn!("found rtl-sdr: {:?}", dev);
        }

        let (ctl, mut reader) = rtlsdr_mt::open(0)?;
        let mut m = Self {
            controller: ctl,
            iq_deque: Arc::new(Mutex::new(VecDeque::new())),
            num_samples_total: Arc::new(Mutex::new(0)),
            num_samples: Arc::new(Mutex::new(0)),
            num_sleep: 0,
        };

        let mut tunes = rtlsdr_mt::TunerGains::default();
        let gains = m.controller.tuner_gains(&mut tunes);
        log::warn!("gain: {:?}", gains);
        let g_max = gains.iter().max().unwrap();

        log::warn!("Using gain: {g_max}");

        //m.controller.enable_agc().expect("Failed to enable agc");
        m.controller
            .set_tuner_gain(*g_max)
            .expect("Failed to enable agc");
        m.controller
            .set_bias_tee(1)
            .expect("Failed to set bias tee");
        m.controller
            .set_center_freq(sig.carrier_hz() as u32)
            .expect("Failed to change center freq");
        m.controller
            .set_sample_rate(fs as u32)
            .expect("Failed to change sample rate");
        m.controller.reset_buffer().expect("Failed to reset buffer");
        let ppm = m.controller.ppm();

        log::warn!("ppm={ppm}");

        let iq_deq = m.iq_deque.clone();
        let num_samples_total = m.num_samples_total.clone();
        let num_samples = m.num_samples.clone();
        thread::spawn(move || {
            loop {
                log::warn!("starting async_read");
                reader
                    .read_async(0, 0, |array| {
                        let mut v = vec![Complex64::default(); array.len()];
                        for i in 0..array.len() / 2 {
                            let re = (array[2 * i] as f64 - 127.3) / 128.0;
                            let im = (array[2 * i + 1] as f64 - 127.3) / 128.0;
                            v[i] = Complex64 { re, im };
                        }

                        let n = v.len();
                        iq_deq.lock().unwrap().push_back(v);
                        *num_samples.lock().unwrap() += n;
                        *num_samples_total.lock().unwrap() += n;
                    })
                    .unwrap();
            }
        });

        Ok(m)
    }
}
