export const PROFILE_SCHEMA = "aeroforge.profile.v1";

export function createProfileFromConfig({ target = "esp32-s3-mini", config, metadata = {} }) {
  if (!Array.isArray(config?.pid) || config.pid.length !== 12) {
    throw new Error("A profile requires 12 PID fields");
  }
  return {
    schema: PROFILE_SCHEMA,
    target,
    createdAt: new Date().toISOString(),
    pid: config.pid.map((value) => Math.trunc(Number(value))),
    mixBias: Number.isFinite(config.mixBias) ? config.mixBias : 1.0,
    metadata,
  };
}

export function validateProfile(profile) {
  if (!profile || profile.schema !== PROFILE_SCHEMA) {
    throw new Error(`Profile schema must be ${PROFILE_SCHEMA}`);
  }
  if (!Array.isArray(profile.pid) || profile.pid.length !== 12) {
    throw new Error("Profile PID array must contain 12 values");
  }
  profile.pid.forEach((value, index) => {
    if (!Number.isInteger(value) || value < -32768 || value > 32767) {
      throw new Error(`PID field ${index} is outside int16 range`);
    }
  });
  if (profile.mixBias != null && !Number.isFinite(Number(profile.mixBias))) {
    throw new Error("Profile mixBias must be finite");
  }
  return true;
}

export function parseProfileText(text) {
  const profile = JSON.parse(text);
  validateProfile(profile);
  return profile;
}

export function formatProfile(profile) {
  validateProfile(profile);
  return `${JSON.stringify(profile, null, 2)}\n`;
}
