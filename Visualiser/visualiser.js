/**
 * Hand IMU Visualiser
 * -------------------
 * Reads glove_data_*.csv from ThesisA.Data
 * Renders a 3D hand skeleton driven by quaternions (per-segment absolute orientation)
 *
 * IMU physical mounting (all segments — fingers and palm):
 *   Y-axis: along bone, pointing PROXIMALLY (towards wrist)
 *   Z-axis: dorsal surface (back of hand / finger)
 *   X-axis: lateral (right when viewed from dorsal)
 *
 * Skeleton convention (Three.js local groups):
 *   Bones extend along -Z (tip at position.z = -length)
 *   So "bone distal direction" = -Z in local space
 *
 * Mounting correction derivation:
 *   We need IMU -Y → skeleton -Z  (both point distally)
 *   Equivalently: IMU +Y → skeleton +Z
 *   That is achieved by Rx(+90°): [w=√2/2, x=√2/2, y=0, z=0]
 *   - IMU X → skel X  (unchanged, lateral)
 *   - IMU Y → skel +Z (was proximal, becomes the +Z axis)
 *   - IMU Z → skel -Y (was dorsal, goes into -Y)
 *   This means at rest (hand flat, DMP ≈ identity), the skeleton
 *   shows fingers extending in -Z (away from camera) which is correct.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ─────────────────────────────────────────────────────────────────────────────
// CONSTANTS
// ─────────────────────────────────────────────────────────────────────────────

const FINGER_NAMES  = ['thumb', 'index', 'middle', 'ring', 'pinky'];
const SEGMENT_TYPES = ['prox', 'mid'];  // proximal then distal (mid = middle phalanx / distal)
const HANDS         = ['left', 'right'];

// Default link lengths in mm (will be overridden by user inputs)
const DEFAULT_LENGTHS = {
  palm_width:  85,
  palm_length: 95,
  thumb_prox:  38,
  thumb_mid:   28,
  index_prox:  43,
  index_mid:   25,
  middle_prox: 48,
  middle_mid:  28,
  ring_prox:   45,
  ring_mid:    27,
  pinky_prox:  35,
  pinky_mid:   22,
  wrist:       55,
};

// MCP (knuckle) positions on the palm as fraction of palm_width offset from centre
// Positive X = towards pinky side (for right hand; mirrored for left)
const MCP_X_OFFSETS = {  // fraction of palm_width/2
  thumb:  -0.85,
  index:  -0.55,
  middle: -0.15,
  ring:    0.25,
  pinky:   0.65,
};

// Colour palette
const COLORS = {
  left:  {
    palm:   0x3b5bdb,
    thumb:  0x74c0fc,
    index:  0x63e6be,
    middle: 0x63e6be,
    ring:   0x63e6be,
    pinky:  0x63e6be,
    wrist:  0x4dabf7,
  },
  right: {
    palm:   0xd63384,
    thumb:  0xff8fa3,
    index:  0xffd43b,
    middle: 0xffd43b,
    ring:   0xffd43b,
    pinky:  0xffd43b,
    wrist:  0xff6b6b,
  },
};

const BONE_RADIUS = 3.5;  // mm
const JOINT_RADIUS = 5.5; // mm
const SCALE = 0.001;      // mm → metres

// ─────────────────────────────────────────────────────────────────────────────
// STATE
// ─────────────────────────────────────────────────────────────────────────────

let renderer, scene, camera, controls;
let hands = {};          // { left: HandModel, right: HandModel }
let frames = [];         // parsed CSV rows
let frameCount = 0;
let currentFrame = 0;
let playing = false;
let playbackSpeed = 1;
let lastTimestamp = null;
let animFrame = null;
let linkLengths = { ...DEFAULT_LENGTHS };
let showImuFrames = true;
let smoothInterp = true;
let gestureLabel = '';

// ─────────────────────────────────────────────────────────────────────────────
// THREE.JS SETUP
// ─────────────────────────────────────────────────────────────────────────────

function initThree() {
  const canvas = document.getElementById('threeCanvas');
  const wrap   = canvas.parentElement;

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setSize(wrap.clientWidth, wrap.clientHeight);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.2;
  renderer.setClearColor(0x0d0f14);

  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x0d0f14, 0.4);

  // Camera
  camera = new THREE.PerspectiveCamera(55, wrap.clientWidth / wrap.clientHeight, 0.001, 20);
  camera.position.set(0, -0.05, 0.45);

  // Orbit controls
  controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.05;
  controls.maxDistance = 5;
  controls.mouseButtons = {
    LEFT:   THREE.MOUSE.ROTATE,
    MIDDLE: THREE.MOUSE.DOLLY,
    RIGHT:  THREE.MOUSE.PAN,
  };

  // Lighting
  const ambLight = new THREE.AmbientLight(0x8899cc, 0.6);
  scene.add(ambLight);
  const dirLight = new THREE.DirectionalLight(0xffffff, 1.8);
  dirLight.position.set(0.3, 0.8, 0.6);
  dirLight.castShadow = true;
  scene.add(dirLight);
  const fillLight = new THREE.DirectionalLight(0x3377ff, 0.4);
  fillLight.position.set(-0.5, -0.2, -0.3);
  scene.add(fillLight);

  // Grid
  const grid = new THREE.GridHelper(2, 20, 0x1c2030, 0x1c2030);
  grid.position.y = -0.20;
  scene.add(grid);

  // Point camera at hands
  controls.target.set(0, -0.05, 0);
  camera.lookAt(0, -0.05, 0);

  // Resize handler
  const ro = new ResizeObserver(() => {
    renderer.setSize(wrap.clientWidth, wrap.clientHeight);
    camera.aspect = wrap.clientWidth / wrap.clientHeight;
    camera.updateProjectionMatrix();
  });
  ro.observe(wrap);

  // Render loop
  function loop(ts) {
    animFrame = requestAnimationFrame(loop);
    controls.update();
    if (playing && frameCount > 1) {
      if (lastTimestamp === null) lastTimestamp = ts;
      const dt = (ts - lastTimestamp) / 1000;
      lastTimestamp = ts;
      const step = dt * playbackSpeed;
      // advance frame time proportionally to sample rate
      const totalDuration = getTotalDuration();
      const frameTime    = getFrameTime(currentFrame);
      let nextTime = frameTime + step;
      if (nextTime >= totalDuration) {
        nextTime = 0;
        currentFrame = 0;
      }
      // find frame matching nextTime
      currentFrame = timeToFrame(nextTime);
      updateScrubber();
      applyFrame(currentFrame);
    }
    renderer.render(scene, camera);
  }
  requestAnimationFrame(loop);
}

// ─────────────────────────────────────────────────────────────────────────────
// HAND MODEL
// ─────────────────────────────────────────────────────────────────────────────

class HandModel {
  constructor(side, scene, linkLengths, colors) {
    this.side   = side;   // 'left' | 'right'
    this.scene  = scene;
    this.root   = new THREE.Group();
    this.root.name = `hand_${side}`;
    scene.add(this.root);
    this.segments = {};  // key: e.g. 'index_prox' → { bone mesh, joint mesh, group }
    this.imuArrows = {};
    this.colors = colors;
    this.build(linkLengths);
  }

  build(ll) {
    // Clear existing
    while (this.root.children.length) this.root.remove(this.root.children[0]);
    this.segments = {};
    this.imuArrows = {};

    const sign = this.side === 'right' ? 1 : -1;
    const pw   = ll.palm_width  * SCALE;
    const pl   = ll.palm_length * SCALE;
    const wl   = ll.wrist * SCALE;

    // ── Wrist ──────────────────────────────────────────────────
    const wristGroup = new THREE.Group();
    wristGroup.name = 'wrist';
    this.root.add(wristGroup);
    const wristMesh = makeCapsule(BONE_RADIUS * SCALE, wl, this.colors[this.side]?.wrist ?? 0x888888);
    wristMesh.rotation.x = Math.PI / 2;
    wristGroup.add(wristMesh);
    // wrist joint ball
    const wj = makeJoint(JOINT_RADIUS * SCALE, this.colors[this.side]?.wrist ?? 0x888888);
    wristGroup.add(wj);
    this.segments['wrist'] = { group: wristGroup, bone: wristMesh, joint: wj };
    addImuArrow(wristGroup, this.imuArrows, 'wrist');

    // ── Palm ──────────────────────────────────────────────────
    const palmGroup = new THREE.Group();
    palmGroup.name = 'palm';
    palmGroup.position.set(0, 0, -wl);  // palm starts at end of wrist
    wristGroup.add(palmGroup);

    // Draw palm as a flat box
    const palmGeo  = new THREE.BoxGeometry(pw, 0.006, pl);
    const palmMat  = new THREE.MeshStandardMaterial({
      color: this.colors[this.side]?.palm ?? 0x334466,
      transparent: true, opacity: 0.35,
      roughness: 0.9, metalness: 0.0,
    });
    const palmMesh = new THREE.Mesh(palmGeo, palmMat);
    palmMesh.position.set(0, 0, -pl / 2);
    palmMesh.castShadow = true;
    palmGroup.add(palmMesh);
    this.segments['palm'] = { group: palmGroup, bone: palmMesh };
    addImuArrow(palmGroup, this.imuArrows, 'palm_mid');

    // ── Fingers ──────────────────────────────────────────────
    FINGER_NAMES.forEach(finger => {
      const mcpX = sign * MCP_X_OFFSETS[finger] * (pw / 2);
      const mcpZ = -(pl * 0.90);  // near distal edge of palm

      const proxLen = ll[`${finger}_prox`] * SCALE;
      const midLen  = ll[`${finger}_mid`]  * SCALE;

      // Proximal phalanx group (attaches at MCP on palm)
      const proxGroup = new THREE.Group();
      proxGroup.name  = `${finger}_prox`;
      proxGroup.position.set(mcpX, 0, mcpZ);
      palmGroup.add(proxGroup);

      const proxBone = makeCapsule(BONE_RADIUS * SCALE, proxLen, this.colors[this.side]?.[finger] ?? 0x88aaff);
      proxBone.position.set(0, 0, -proxLen / 2);
      proxBone.rotation.x = 0;
      proxGroup.add(proxBone);
      const proxJoint = makeJoint(JOINT_RADIUS * SCALE, this.colors[this.side]?.[finger] ?? 0x88aaff);
      proxGroup.add(proxJoint);
      addImuArrow(proxGroup, this.imuArrows, `${finger}_prox`);

      // Distal phalanx group (attaches at PIP joint, end of proximal)
      const midGroup = new THREE.Group();
      midGroup.name  = `${finger}_mid`;
      midGroup.position.set(0, 0, -proxLen);
      proxGroup.add(midGroup);

      const midBone = makeCapsule(BONE_RADIUS * SCALE * 0.85, midLen, this.colors[this.side]?.[finger] ?? 0x88aaff);
      midBone.position.set(0, 0, -midLen / 2);
      midGroup.add(midBone);
      const midJoint = makeJoint(JOINT_RADIUS * SCALE * 0.85, this.colors[this.side]?.[finger] ?? 0x88aaff);
      midGroup.add(midJoint);
      addImuArrow(midGroup, this.imuArrows, `${finger}_mid`);

      // Fingertip cap
      const tipGeo = new THREE.SphereGeometry(BONE_RADIUS * SCALE * 0.85, 8, 6);
      const tipMat = new THREE.MeshStandardMaterial({ color: this.colors[this.side]?.[finger] ?? 0x88aaff, roughness: 0.6 });
      const tip    = new THREE.Mesh(tipGeo, tipMat);
      tip.position.set(0, 0, -midLen);
      midGroup.add(tip);

      this.segments[`${finger}_prox`] = { group: proxGroup, bone: proxBone, joint: proxJoint };
      this.segments[`${finger}_mid`]  = { group: midGroup,  bone: midBone,  joint: midJoint };
    });
  }

  /**
   * Apply one frame of quaternion data.
   * quats: { wrist, palm_mid, palm_prox, thumb_prox, thumb_mid, index_prox, index_mid, ... }
   * Each value is a THREE.Quaternion representing absolute IMU orientation.
   *
   * Strategy: use the quaternion from the segment's own IMU directly as the
   * world-space orientation of that group.  We convert to the group's local
   * frame by pre-multiplying by the inverse of the parent's world quaternion.
   */
  applyQuaternions(quats) {
    // Wrist — absolute orientation in world space
    if (quats.wrist) {
      this.root.quaternion.copy(quats.wrist);
    }

    const applySegment = (key, parentWorldQ) => {
      const seg = this.segments[key];
      if (!seg) return;
      const q = quats[key];
      if (!q) return;
      // local = inverse(parentWorld) * world
      const parentInv = parentWorldQ.clone().invert();
      const localQ    = parentInv.multiply(q);
      seg.group.quaternion.copy(localQ);
    };

    const wristWorldQ = this.root.quaternion.clone();
    applySegment('palm',       wristWorldQ);

    const palmWorldQ = new THREE.Quaternion();
    this.segments['palm']?.group.getWorldQuaternion(palmWorldQ);

    FINGER_NAMES.forEach(finger => {
      applySegment(`${finger}_prox`, palmWorldQ);
      const proxWorldQ = new THREE.Quaternion();
      this.segments[`${finger}_prox`]?.group.getWorldQuaternion(proxWorldQ);
      applySegment(`${finger}_mid`, proxWorldQ);
    });
  }

  setImuFramesVisible(v) {
    Object.values(this.imuArrows).forEach(a => {
      if (a) a.visible = v;
    });
  }

  rebuild(ll) {
    this.build(ll);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// GEOMETRY HELPERS
// ─────────────────────────────────────────────────────────────────────────────

function makeCapsule(radius, length, color) {
  // Approximate capsule with cylinder + hemisphere caps
  const geo = new THREE.CylinderGeometry(radius, radius, length, 10, 1, false);
  geo.rotateX(Math.PI / 2);  // align along Z
  const mat = new THREE.MeshStandardMaterial({
    color, roughness: 0.5, metalness: 0.1,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = true;
  return mesh;
}

function makeJoint(radius, color) {
  const geo  = new THREE.SphereGeometry(radius, 10, 8);
  const mat  = new THREE.MeshStandardMaterial({ color, roughness: 0.3, metalness: 0.2 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = true;
  return mesh;
}

function addImuArrow(group, arrowMap, key) {
  // Small RGB axes (X=red, Y=green, Z=blue) representing IMU frame
  const len = 0.012;
  const arrowGroup = new THREE.Group();
  arrowGroup.name = `imu_${key}`;
  arrowGroup.visible = true;

  const addArrow = (dir, color) => {
    const origin = new THREE.Vector3(0, 0, 0);
    const arrow = new THREE.ArrowHelper(dir, origin, len, color, len * 0.35, len * 0.25);
    arrowGroup.add(arrow);
  };
  addArrow(new THREE.Vector3(1, 0, 0), 0xff4444);
  addArrow(new THREE.Vector3(0, 1, 0), 0x44ff44);
  addArrow(new THREE.Vector3(0, 0, 1), 0x4444ff);

  group.add(arrowGroup);
  arrowMap[key] = arrowGroup;
}

// ─────────────────────────────────────────────────────────────────────────────
// CSV PARSING
// ─────────────────────────────────────────────────────────────────────────────

function parseCSV(text) {
  const lines   = text.split('\n').filter(l => l.trim());
  const headers = lines[0].split(',').map(h => h.trim());
  const rows    = [];

  for (let i = 1; i < lines.length; i++) {
    const vals = splitCSVLine(lines[i]);
    if (vals.length < headers.length) continue;
    const row = {};
    headers.forEach((h, j) => {
      row[h] = vals[j];
    });
    rows.push(row);
  }
  return rows;
}

function splitCSVLine(line) {
  // Simple CSV split (handles no quoted commas in this dataset)
  return line.split(',').map(v => v.trim());
}

function extractQuats(row, side) {
  /**
   * Returns { wrist, palm, thumb_prox, thumb_mid, index_prox,
   *           index_mid, middle_prox, middle_mid, ring_prox, ring_mid,
   *           pinky_prox, pinky_mid }
   * Each value is a THREE.Quaternion in skeleton local convention.
   *
   * The MPU6050 DMP outputs world-frame absolute orientation quaternions.
   * We post-multiply by mountCorr to convert from IMU body frame
   * (Y-proximal, Z-dorsal) into the skeleton frame (-Z = bone distal).
   *
   * Mounting correction: Rx(+90°)
   *   w = cos(π/4) = √2/2 ≈ 0.7071
   *   x = sin(π/4) = √2/2 ≈ 0.7071
   *   y = 0, z = 0
   *
   * This maps: IMU Y (proximal) → skeleton +Z,
   *            IMU -Y (distal)  → skeleton -Z  (bone direction)
   * Same correction applied to ALL segments (fingers, palm, wrist) since
   * they all share the same physical mounting convention.
   */

  const SQ2_2 = Math.SQRT2 / 2;   // ≈ 0.7071
  const mountCorr = new THREE.Quaternion(SQ2_2, 0, 0, SQ2_2); // THREE.Quaternion(x,y,z,w)

  const getQ = (prefix) => {
    const w = parseFloat(row[`${prefix}_quat_w`]);
    const x = parseFloat(row[`${prefix}_quat_x`]);
    const y = parseFloat(row[`${prefix}_quat_y`]);
    const z = parseFloat(row[`${prefix}_quat_z`]);
    if (isNaN(w) || isNaN(x) || isNaN(y) || isNaN(z)) return null;
    const q = new THREE.Quaternion(x, y, z, w);
    q.normalize();
    q.multiply(mountCorr);
    return q;
  };

  const quats = {};
  quats['wrist']       = getQ(`${side}_wrist`);
  quats['palm']        = getQ(`${side}_palm_mid`);  // back-of-hand IMU
  FINGER_NAMES.forEach(f => {
    quats[`${f}_prox`] = getQ(`${side}_${f}_prox`);
    quats[`${f}_mid`]  = getQ(`${side}_${f}_mid`);
  });
  return quats;
}

// ─────────────────────────────────────────────────────────────────────────────
// FRAME TIME HELPERS
// ─────────────────────────────────────────────────────────────────────────────

function getFrameTime(idx) {
  if (!frames.length) return 0;
  const f = frames[Math.min(idx, frames.length - 1)];
  // Use right_time (ms) as primary timeline
  const t = parseFloat(f.right_time) || parseFloat(f.left_time) || 0;
  return t / 1000;  // → seconds
}

function getTotalDuration() {
  return getFrameTime(frames.length - 1);
}

function timeToFrame(t) {
  // Binary search for closest frame
  let lo = 0, hi = frames.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (getFrameTime(mid) < t) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

// ─────────────────────────────────────────────────────────────────────────────
// APPLY FRAME TO SCENE
// ─────────────────────────────────────────────────────────────────────────────

function applyFrame(idx) {
  if (!frames.length) return;
  const row = frames[idx];

  HANDS.forEach(side => {
    const hand = hands[side];
    if (!hand) return;

    let quats = extractQuats(row, side);

    // Optional SLERP interpolation between adjacent frames
    if (smoothInterp && idx + 1 < frames.length) {
      const row2   = frames[idx + 1];
      const quats2 = extractQuats(row2, side);
      const t1 = getFrameTime(idx);
      const t2 = getFrameTime(idx + 1);
      const dt = t2 - t1;
      // Alpha: how far between frames (0–1).  For display purposes use 0.5
      // unless doing time-accurate interpolation (requires exact timestamp).
      const alpha = dt > 0 ? 0.5 : 0;
      if (alpha > 0) {
        Object.keys(quats).forEach(k => {
          if (quats[k] && quats2[k]) {
            quats[k].slerp(quats2[k], alpha);
          }
        });
      }
    }

    hand.applyQuaternions(quats);
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// PLAYBACK UI HELPERS
// ─────────────────────────────────────────────────────────────────────────────

function updateScrubber() {
  const scrubber = document.getElementById('scrubber');
  const timeNow  = document.getElementById('timeNow');
  const timeTot  = document.getElementById('timeTot');
  const tot = getTotalDuration();
  const cur = getFrameTime(currentFrame);
  scrubber.value = Math.round((cur / tot) * 1000);
  timeNow.textContent = cur.toFixed(2) + 's';
  timeTot.textContent = tot.toFixed(2) + 's';
  document.getElementById('frameInfo').textContent =
    `Frame ${currentFrame + 1} / ${frameCount}  ·  ${cur.toFixed(3)}s`;
}

function setPlaying(v) {
  playing = v;
  lastTimestamp = null;
  document.getElementById('iconPlay').style.display  = v ? 'none' : '';
  document.getElementById('iconPause').style.display = v ? '' : 'none';
}

// ─────────────────────────────────────────────────────────────────────────────
// LOAD FILE
// ─────────────────────────────────────────────────────────────────────────────

function loadFile(file) {
  const name = file.name;
  // Extract gesture label from filename convention: glove_data_{gesture}_{run}_{date}.csv
  const m = name.match(/glove_data_(.+?)_\d+s_\d+/);
  gestureLabel = m ? m[1].replace(/_/g, ' ') : name.replace('.csv', '');
  document.getElementById('gestureName').textContent = gestureLabel;

  const reader = new FileReader();
  reader.onload = (e) => {
    frames = parseCSV(e.target.result);
    frameCount = frames.length;

    if (!frameCount) {
      alert('CSV parsed but no data rows found — check file format.');
      return;
    }

    // Build hand models
    Object.values(hands).forEach(h => { h.rebuild(linkLengths); });

    // Apply first frame
    currentFrame = 0;
    applyFrame(0);

    // Show UI
    document.getElementById('dropOverlay').classList.add('hidden');
    document.getElementById('playbar').style.display = '';

    const scrubber = document.getElementById('scrubber');
    scrubber.max = 1000;
    scrubber.value = 0;
    updateScrubber();
    setPlaying(false);
  };
  reader.readAsText(file);
}

// ─────────────────────────────────────────────────────────────────────────────
// BUILD INITIAL SCENE  (both hands at rest)
// ─────────────────────────────────────────────────────────────────────────────

function buildHands() {
  // Position left hand to the left, right to the right
  // Side-by-side with a gap between them

  if (hands.left)  { scene.remove(hands.left.root);  }
  if (hands.right) { scene.remove(hands.right.root); }

  hands.left  = new HandModel('left',  scene, linkLengths, COLORS);
  hands.right = new HandModel('right', scene, linkLengths, COLORS);

  // Offset them so they sit side by side
  hands.left.root.position.set(-0.11, 0, 0);
  hands.right.root.position.set(0.11, 0, 0);

  // Mirror the left hand along X axis so both palms face viewer
  hands.left.root.scale.x = -1;

  // Set initial identity pose (hands flat, fingers extended)
  HANDS.forEach(side => {
    const hand = hands[side];
    const identityQuats = {};
    identityQuats['wrist'] = new THREE.Quaternion();
    identityQuats['palm']  = new THREE.Quaternion();
    FINGER_NAMES.forEach(f => {
      identityQuats[`${f}_prox`] = new THREE.Quaternion();
      identityQuats[`${f}_mid`]  = new THREE.Quaternion();
    });
    hand.applyQuaternions(identityQuats);
    hand.setImuFramesVisible(showImuFrames);
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// READ LINK LENGTHS FROM DOM
// ─────────────────────────────────────────────────────────────────────────────

function readLinkLengths() {
  const ids = [
    'palm_width', 'palm_length', 'wrist',
    'thumb_prox', 'thumb_mid',
    'index_prox', 'index_mid',
    'middle_prox', 'middle_mid',
    'ring_prox', 'ring_mid',
    'pinky_prox', 'pinky_mid',
  ];
  ids.forEach(id => {
    const el = document.getElementById(`ll_${id}`);
    if (el) linkLengths[id] = parseFloat(el.value) || DEFAULT_LENGTHS[id];
  });
}

function resetLinkLengths() {
  linkLengths = { ...DEFAULT_LENGTHS };
  Object.entries(DEFAULT_LENGTHS).forEach(([k, v]) => {
    const el = document.getElementById(`ll_${k}`);
    if (el) el.value = v;
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// EVENT WIRING
// ─────────────────────────────────────────────────────────────────────────────

function wireUI() {
  // Settings panel toggle
  document.getElementById('toggleSettings').addEventListener('click', () => {
    document.getElementById('settingsPanel').classList.toggle('collapsed');
  });

  // Apply link lengths
  document.getElementById('applyLengths').addEventListener('click', () => {
    readLinkLengths();
    buildHands();
    if (frames.length) applyFrame(currentFrame);
  });

  // Reset lengths
  document.getElementById('resetLengths').addEventListener('click', () => {
    resetLinkLengths();
    buildHands();
    if (frames.length) applyFrame(currentFrame);
  });

  // Play/Pause
  document.getElementById('playBtn').addEventListener('click', () => {
    setPlaying(!playing);
  });

  // Scrubber drag
  const scrubber = document.getElementById('scrubber');
  scrubber.addEventListener('input', () => {
    setPlaying(false);
    const tot = getTotalDuration();
    const t   = (parseInt(scrubber.value) / 1000) * tot;
    currentFrame = timeToFrame(t);
    applyFrame(currentFrame);
    updateScrubber();
  });

  // Playback speed
  document.getElementById('playbackSpeed').addEventListener('change', (e) => {
    playbackSpeed = parseFloat(e.target.value);
  });

  // Show axes
  document.getElementById('showAxes').addEventListener('change', (e) => {
    const axesHelper = scene.getObjectByName('worldAxes');
    if (e.target.checked) {
      if (!axesHelper) {
        const ax = new THREE.AxesHelper(0.15);
        ax.name = 'worldAxes';
        scene.add(ax);
      }
    } else {
      if (axesHelper) scene.remove(axesHelper);
    }
  });

  // Show IMU frames
  document.getElementById('showImuFrames').addEventListener('change', (e) => {
    showImuFrames = e.target.checked;
    HANDS.forEach(side => hands[side]?.setImuFramesVisible(showImuFrames));
  });

  // Smooth interpolation
  document.getElementById('smoothInterp').addEventListener('change', (e) => {
    smoothInterp = e.target.checked;
  });

  // Reset camera
  document.getElementById('resetCamBtn').addEventListener('click', () => {
    camera.position.set(0, -0.05, 0.45);
    controls.reset();
  });

  // File browse
  const fileInput = document.getElementById('fileInput');
  document.getElementById('browseBtn').addEventListener('click', () => fileInput.click());
  fileInput.addEventListener('change', (e) => {
    if (e.target.files[0]) loadFile(e.target.files[0]);
  });

  // Drag & drop
  const dropOverlay = document.getElementById('dropOverlay');
  const dropCard    = dropOverlay.querySelector('.drop-card');
  const viewport    = document.querySelector('.viewport-wrap');

  viewport.addEventListener('dragover', (e) => {
    e.preventDefault();
    dropCard.classList.add('drag-over');
  });
  viewport.addEventListener('dragleave', () => {
    dropCard.classList.remove('drag-over');
  });
  viewport.addEventListener('drop', (e) => {
    e.preventDefault();
    dropCard.classList.remove('drag-over');
    const file = e.dataTransfer.files[0];
    if (file && file.name.endsWith('.csv')) loadFile(file);
  });

  // Keyboard shortcuts
  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT') return;
    if (e.code === 'Space') { e.preventDefault(); setPlaying(!playing); }
    if (e.code === 'ArrowRight') {
      setPlaying(false);
      currentFrame = Math.min(currentFrame + 1, frameCount - 1);
      applyFrame(currentFrame);
      updateScrubber();
    }
    if (e.code === 'ArrowLeft') {
      setPlaying(false);
      currentFrame = Math.max(currentFrame - 1, 0);
      applyFrame(currentFrame);
      updateScrubber();
    }
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// ENTRY POINT
// ─────────────────────────────────────────────────────────────────────────────

initThree();
buildHands();
wireUI();
