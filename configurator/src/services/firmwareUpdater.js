const MAX_FIRMWARE_BYTES = 2 * 1024 * 1024;

export function validateFirmwareFile(file) {
  if (!file) {
    throw new Error("Select a firmware .bin file first");
  }
  if (!/\.bin$/i.test(file.name ?? "")) {
    throw new Error("Firmware file must use the .bin extension");
  }
  if (!Number.isFinite(file.size) || file.size <= 0) {
    throw new Error("Firmware file is empty");
  }
  if (file.size > MAX_FIRMWARE_BYTES) {
    throw new Error(`Firmware file exceeds ${MAX_FIRMWARE_BYTES} bytes`);
  }
  return true;
}

export async function prepareFirmwareFile(file) {
  validateFirmwareFile(file);
  const bytes = new Uint8Array(await file.arrayBuffer());
  let checksum = 0;
  for (const byte of bytes) {
    checksum = (checksum + byte) >>> 0;
  }
  return {
    metadata: {
      name: file.name,
      size: bytes.byteLength,
      checksum,
      schema: "aeroforge.firmware.v1",
    },
    bytes,
  };
}
