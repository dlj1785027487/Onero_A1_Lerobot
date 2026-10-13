"""Timed initialization with a completion barrier before caller-owned cleanup."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Iterator, Mapping
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager


@contextmanager
def startup_stage(logger: logging.Logger, name: str) -> Iterator[None]:
    started = time.monotonic()
    logger.info("Startup: %s ...", name)
    try:
        yield
    except BaseException:
        logger.exception("Startup: %s failed after %.3fs", name, time.monotonic() - started)
        raise
    else:
        logger.info("Startup: %s ready in %.3fs", name, time.monotonic() - started)


def run_startup_tasks(
    logger: logging.Logger,
    tasks: Mapping[str, Callable[[], None]],
    *,
    parallel: bool = True,
) -> None:
    """Wait for every submitted operation, including on failure or interruption.

    Tasks only initialize their own device. The caller may clean up shared
    session resources after this function returns or raises, never from a task.
    """
    def run(name: str, operation: Callable[[], None]) -> None:
        with startup_stage(logger, name):
            operation()

    if not parallel or len(tasks) < 2:
        for name, operation in tasks.items():
            run(name, operation)
        return

    errors: list[BaseException] = []
    # Executor shutdown waits for all submitted tasks even if submission or
    # as_completed is interrupted. No device can be closed while it connects.
    with ThreadPoolExecutor(max_workers=len(tasks), thread_name_prefix="a1_startup") as pool:
        futures = [pool.submit(run, name, operation) for name, operation in tasks.items()]
        for future in as_completed(futures):
            try:
                future.result()
            except BaseException as exc:
                errors.append(exc)
    if errors:
        raise errors[0]
