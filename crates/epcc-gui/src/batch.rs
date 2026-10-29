//! Runs a batch of conversions in the background and reports back to the window.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, channel};

use eframe::egui;
use epcc_core::{ConversionJob, Error, Outcome, Progress, Settings, run_batch};

/// How one file's conversion ended.
pub enum JobEnd {
    Converted { outputs: Vec<PathBuf>, points_written: u64, warnings: Vec<String> },
    Skipped(PathBuf),
    Failed(String),
    Cancelled,
}

impl JobEnd {
    fn from_result(result: &epcc_core::Result<Outcome>) -> JobEnd {
        match result {
            Ok(Outcome::Converted(report)) => JobEnd::Converted {
                outputs: report.outputs.clone(),
                points_written: report.points_written,
                warnings: report.warnings.clone(),
            },
            Ok(Outcome::Skipped(existing)) => JobEnd::Skipped(existing.clone()),
            Err(Error::Cancelled) => JobEnd::Cancelled,
            Err(error) => JobEnd::Failed(error.to_string()),
        }
    }
}

pub struct Batch {
    /// For each job: the file list row it belongs to, and its progress.
    pub jobs: Vec<(usize, Arc<Progress>)>,
    ended: Receiver<(usize, JobEnd)>,
    remaining: usize,
}

impl Batch {
    /// Starts converting. `jobs` pairs each job with its file list row.
    pub fn start(jobs: Vec<(usize, ConversionJob)>, settings: Settings, ctx: egui::Context) -> Batch {
        let (rows, jobs): (Vec<usize>, Vec<ConversionJob>) = jobs.into_iter().unzip();
        let progress: Vec<Arc<Progress>> = jobs.iter().map(|_| Arc::new(Progress::new())).collect();
        let (sender, ended) = channel();

        let thread_progress = progress.clone();
        let thread_rows = rows.clone();
        std::thread::spawn(move || {
            run_batch(&jobs, &settings, &thread_progress, &|index, result| {
                let _ = sender.send((thread_rows[index], JobEnd::from_result(result)));
                ctx.request_repaint();
            });
        });

        Batch { remaining: rows.len(), jobs: rows.into_iter().zip(progress).collect(), ended }
    }

    /// Jobs that ended since the last call, by file list row.
    pub fn take_ended(&mut self) -> Vec<(usize, JobEnd)> {
        let ended: Vec<_> = self.ended.try_iter().collect();
        self.remaining -= ended.len();
        ended
    }

    pub fn is_finished(&self) -> bool {
        self.remaining == 0
    }

    pub fn cancel(&self) {
        for (_, progress) in &self.jobs {
            progress.cancel();
        }
    }

    pub fn is_cancelling(&self) -> bool {
        self.jobs.first().is_some_and(|(_, progress)| progress.is_cancelled())
    }

    /// Overall progress from 0 to 1.
    pub fn fraction(&self) -> f32 {
        let sum: f32 = self.jobs.iter().map(|(_, progress)| progress.fraction()).sum();
        sum / self.jobs.len().max(1) as f32
    }

    pub fn progress_of(&self, row: usize) -> Option<f32> {
        self.jobs.iter().find(|(job_row, _)| *job_row == row).map(|(_, progress)| progress.fraction())
    }
}
