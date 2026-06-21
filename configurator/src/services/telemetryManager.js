import { TelemetryGroup } from "./protocol.js";

const pageGroups = new Map([
  ["Dashboard", TelemetryGroup.DASHBOARD],
  ["Receiver", TelemetryGroup.RECEIVER],
  ["Motors", TelemetryGroup.MOTORS],
  ["Sensors", TelemetryGroup.SENSORS],
  ["Compass", TelemetryGroup.SENSORS],
  ["Filters and FFT", TelemetryGroup.FILTERS],
  ["GPS and Navigation", TelemetryGroup.GPS],
  ["System", TelemetryGroup.SYSTEM],
  ["Telemetry", TelemetryGroup.DASHBOARD],
]);

export function telemetryGroupForPage(page) {
  return pageGroups.get(page) ?? TelemetryGroup.DASHBOARD;
}

export function telemetryRateForPage(page) {
  if (page === "Dashboard" || page === "Receiver" || page === "Motors") return 10;
  if (page === "Filters and FFT" || page === "Telemetry") return 5;
  return 2;
}

export async function updateTelemetrySubscription(client, page) {
  if (!client?.isConnected?.()) return null;
  const group = telemetryGroupForPage(page);
  const rateHz = telemetryRateForPage(page);
  await client.subscribeTelemetry(group, rateHz);
  return { group, rateHz };
}
