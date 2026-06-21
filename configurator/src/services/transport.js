export class Transport {
  constructor(name) {
    this.name = name;
    this.dataHandlers = new Set();
    this.errorHandlers = new Set();
    this.stateHandlers = new Set();
    this.connected = false;
  }

  async discover() {
    return [];
  }

  async connect() {
    throw new Error(`${this.name} connect() is not implemented`);
  }

  async disconnect() {
    this.setConnected(false);
  }

  async write() {
    throw new Error(`${this.name} write() is not implemented`);
  }

  onData(callback) {
    this.dataHandlers.add(callback);
    return () => this.dataHandlers.delete(callback);
  }

  onError(callback) {
    this.errorHandlers.add(callback);
    return () => this.errorHandlers.delete(callback);
  }

  onStateChange(callback) {
    this.stateHandlers.add(callback);
    return () => this.stateHandlers.delete(callback);
  }

  isConnected() {
    return this.connected;
  }

  emitData(data) {
    for (const handler of this.dataHandlers) {
      handler(data);
    }
  }

  emitError(error) {
    for (const handler of this.errorHandlers) {
      handler(error);
    }
  }

  setConnected(connected, detail = {}) {
    this.connected = connected;
    for (const handler of this.stateHandlers) {
      handler({ connected, ...detail });
    }
  }
}
