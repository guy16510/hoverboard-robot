// SPDX-License-Identifier: GPL-3.0-only

export class CaptureLifecycle {
  #finishing = false;
  #finalized = false;

  beginFinish() {
    if (this.#finishing || this.#finalized) return false;
    this.#finishing = true;
    return true;
  }

  markFinalized() {
    this.#finalized = true;
  }

  get canRecord() {
    return !this.#finalized;
  }

  get canHandleSerialEvent() {
    return !this.#finalized;
  }
}
