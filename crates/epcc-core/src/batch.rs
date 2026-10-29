//! Converting many files, several at a time.

use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;

use crate::output::ClaimedPaths;
use crate::{ConversionJob, Outcome, Progress, Result, Settings, convert};

/// Converts every job and returns their results in job order. `progress[i]` belongs to
/// `jobs[i]`; `finished` is called as each job ends, from the thread that ran it.
pub fn run_batch(
    jobs: &[ConversionJob],
    settings: &Settings,
    progress: &[impl AsRef<Progress> + Sync],
    finished: &(dyn Fn(usize, &Result<Outcome>) + Sync),
) -> Vec<Result<Outcome>> {
    assert_eq!(jobs.len(), progress.len(), "one progress per job");
    let claimed = ClaimedPaths::default();
    let next_job = AtomicUsize::new(0);
    let results: Mutex<Vec<Option<Result<Outcome>>>> = Mutex::new(jobs.iter().map(|_| None).collect());

    thread::scope(|scope| {
        for _ in 0..parallel_files(settings, jobs.len()) {
            scope.spawn(|| {
                loop {
                    let index = next_job.fetch_add(1, Ordering::Relaxed);
                    let Some(job) = jobs.get(index) else { break };
                    let job_progress = progress[index].as_ref();
                    let result = if job_progress.is_cancelled() {
                        Err(crate::Error::Cancelled)
                    } else {
                        convert(job, settings, job_progress, &claimed)
                    };
                    finished(index, &result);
                    results.lock().unwrap_or_else(|poisoned| poisoned.into_inner())[index] = Some(result);
                }
            });
        }
    });

    results
        .into_inner()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .into_iter()
        .map(|result| result.expect("every job ran"))
        .collect()
}

/// Each conversion already uses all cores where its formats allow. Running a few files at once
/// still helps, because some formats (E57, LAS) decode on a single core.
fn parallel_files(settings: &Settings, job_count: usize) -> usize {
    let configured = settings.general.parallel_files;
    let automatic = thread::available_parallelism().map_or(1, |cores| (cores.get() / 4).clamp(1, 4));
    let wanted = if configured == 0 { automatic } else { configured };
    wanted.min(job_count).max(1)
}
