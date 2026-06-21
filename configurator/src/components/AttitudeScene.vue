<script setup>
import { onBeforeUnmount, onMounted, ref, watch } from "vue";
import * as THREE from "three";

const props = defineProps({
  roll: { type: Number, default: 0 },
  pitch: { type: Number, default: 0 },
  yaw: { type: Number, default: 0 },
});

const host = ref(null);
let renderer;
let scene;
let camera;
let craft;
let raf = 0;
let resizeObserver;
let lastTs = 0;
let dirty = true;

// Telemetry arrives at ~4 Hz; rendering at that rate looks like the model is
// teleporting. We keep a `target` attitude (from telemetry) and a `shown`
// attitude that eases toward it every animation frame, so motion is smooth at
// the display refresh rate. The loop parks itself once `shown` has caught up
// (no telemetry change), so a still bench costs zero GPU.
const target = { x: 0, y: 0, z: 0 }; // radians
const shown = { x: 0, y: 0, z: 0 };  // radians
const SETTLE_RAD = 0.0006;           // ~0.034 deg: below this, stop animating
const FOLLOW_K = 13;                  // ease rate (higher = snappier)

const deg = (d) => THREE.MathUtils.degToRad(d);

function makeMotor(x, z, color) {
  const group = new THREE.Group();
  const hub = new THREE.Mesh(
    new THREE.CylinderGeometry(0.15, 0.15, 0.08, 16),
    new THREE.MeshStandardMaterial({ color }),
  );
  hub.rotation.x = Math.PI / 2;
  const disc = new THREE.Mesh(
    new THREE.TorusGeometry(0.28, 0.015, 6, 24),
    new THREE.MeshStandardMaterial({ color: 0xdce7de }),
  );
  disc.rotation.x = Math.PI / 2;
  group.add(hub, disc);
  group.position.set(x, 0, z);
  return group;
}

function buildCraft() {
  const group = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.9, 0.18, 1.0),
    new THREE.MeshStandardMaterial({ color: 0x2ad184, roughness: 0.45 }),
  );
  const front = new THREE.Mesh(
    new THREE.ConeGeometry(0.22, 0.5, 16),
    new THREE.MeshStandardMaterial({ color: 0xe9f0ea }),
  );
  front.rotation.x = Math.PI / 2;
  front.position.z = -0.72;

  const armMaterial = new THREE.MeshStandardMaterial({ color: 0x8e9991, roughness: 0.6 });
  const armA = new THREE.Mesh(new THREE.BoxGeometry(2.8, 0.08, 0.08), armMaterial);
  const armB = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.08, 2.8), armMaterial);

  group.add(body, front, armA, armB);
  group.add(makeMotor(-1.35, -1.35, 0xe9f0ea));
  group.add(makeMotor(1.35, -1.35, 0xe9f0ea));
  group.add(makeMotor(-1.35, 1.35, 0x2ad184));
  group.add(makeMotor(1.35, 1.35, 0x2ad184));
  return group;
}

function resize() {
  if (!host.value || !renderer || !camera) return;
  const { clientWidth, clientHeight } = host.value;
  renderer.setSize(clientWidth, clientHeight, false);
  camera.aspect = clientWidth / Math.max(1, clientHeight);
  camera.updateProjectionMatrix();
  dirty = true;
  ensureLoop();
}

function syncTarget() {
  target.z = deg(-props.roll);
  target.x = deg(props.pitch);
  target.y = deg(props.yaw);
  ensureLoop();
}

// Shortest signed angular step from cur toward tgt (handles 359->0 wrap).
function stepAngle(cur, tgt, alpha) {
  let d = tgt - cur;
  d = Math.atan2(Math.sin(d), Math.cos(d));
  return { value: cur + d * alpha, remaining: Math.abs(d) };
}

function frame(ts) {
  raf = 0;
  const dt = lastTs ? Math.min(0.05, (ts - lastTs) / 1000) : 0.016;
  lastTs = ts;
  // Frame-rate-independent easing: same feel at 30 or 144 Hz.
  const alpha = 1 - Math.exp(-FOLLOW_K * dt);

  const rx = stepAngle(shown.x, target.x, alpha);
  const ry = stepAngle(shown.y, target.y, alpha);
  const rz = stepAngle(shown.z, target.z, alpha);
  shown.x = rx.value;
  shown.y = ry.value;
  shown.z = rz.value;
  craft.rotation.set(shown.x, shown.y, shown.z);
  renderer.render(scene, camera);

  const settled = rx.remaining < SETTLE_RAD && ry.remaining < SETTLE_RAD && rz.remaining < SETTLE_RAD;
  if (settled && !dirty) {
    shown.x = target.x;
    shown.y = target.y;
    shown.z = target.z;
    lastTs = 0;
    return; // park the loop until the next telemetry change / resize
  }
  dirty = false;
  raf = requestAnimationFrame(frame);
}

function ensureLoop() {
  if (!raf && renderer && scene && camera) {
    lastTs = 0;
    raf = requestAnimationFrame(frame);
  }
}

onMounted(() => {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111411);
  camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
  camera.position.set(3.8, 2.5, 4.2);
  camera.lookAt(0, 0, 0);

  renderer = new THREE.WebGLRenderer({
    antialias: true,
    powerPreference: "high-performance",
  });
  // Crisp on HiDPI without paying for >2x oversampling.
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  host.value.appendChild(renderer.domElement);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x202820, 2.2));
  const key = new THREE.DirectionalLight(0xffffff, 2.5);
  key.position.set(3, 5, 4);
  scene.add(key);

  const grid = new THREE.GridHelper(5, 8, 0x3b443d, 0x242a25);
  grid.position.y = -0.55;
  scene.add(grid);

  craft = buildCraft();
  // Start already at the current attitude so the model doesn't sweep in from 0.
  syncTarget();
  shown.x = target.x;
  shown.y = target.y;
  shown.z = target.z;
  craft.rotation.set(shown.x, shown.y, shown.z);
  scene.add(craft);

  resize();
  window.addEventListener("resize", resize);
  resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(host.value);
  ensureLoop();
});

watch(() => [props.roll, props.pitch, props.yaw], syncTarget);

onBeforeUnmount(() => {
  cancelAnimationFrame(raf);
  raf = 0;
  window.removeEventListener("resize", resize);
  resizeObserver?.disconnect();
  scene?.traverse((object) => {
    object.geometry?.dispose?.();
    if (Array.isArray(object.material)) {
      object.material.forEach((material) => material.dispose?.());
    } else {
      object.material?.dispose?.();
    }
  });
  renderer?.dispose();
});
</script>

<template>
  <div ref="host" class="attitude-scene" aria-label="3D quadcopter attitude view"></div>
</template>
