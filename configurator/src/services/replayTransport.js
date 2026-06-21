import { Transport } from "./transport.js";

export class ReplayTransport extends Transport {
  constructor(frames = []) {
    super("Replay");
    this.frames = frames;
  }

  async connect() {
    this.setConnected(true, { path: "REPLAY" });
    for (const frame of this.frames) {
      queueMicrotask(() => this.emitData(frame));
    }
  }

  async write() {
    if (!this.connected) {
      throw new Error("Replay transport is not connected");
    }
  }
}
