/**
 * Autonomous Bottle-Cap Sorting Machine
 * Interactive Application & Simulation Engine
 */

document.addEventListener('DOMContentLoaded', () => {
  // Initialize Lucide icons if available
  if (window.lucide) {
    window.lucide.createIcons();
  }

  initSimulator();
  initPosterModal();
  initVisionPipeline();
  initPinoutTable();
  initBOMTable();
  initCodeVault();
  initSmoothScroll();
});

/* ==========================================================================
   1. Gantry Pick & Place 2D Simulator
   ========================================================================== */
function initSimulator() {
  const canvas = document.getElementById('simulation-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  // Physical specs
  const BOARD_SIZE_MM = 300;
  const GRID_DIVS = 10;
  const CELL_SIZE_MM = 30;
  const CAP_RADIUS_MM = 15;

  // Viewport / canvas mapping
  // Let canvas size be 580 x 500 (leaving space on right for 4 bins)
  const PADDING_LEFT = 40;
  const PADDING_TOP = 40;
  const GRID_PIXELS = 360;
  const SCALE = GRID_PIXELS / BOARD_SIZE_MM; // 1.2 px per mm

  // Bins layout (on right side of grid, 300mm to 360mm range)
  const BINS = [
    { id: 'red', name: 'Bin 1 (Red)', color: '#ef4444', x: 335, y: 35, w: 45, h: 45, count: 0 },
    { id: 'yellow', name: 'Bin 2 (Yellow)', color: '#eab308', x: 335, y: 110, w: 45, h: 45, count: 0 },
    { id: 'green', name: 'Bin 3 (Green)', color: '#10b981', x: 335, y: 185, w: 45, h: 45, count: 0 },
    { id: 'blue', name: 'Bin 4 (Blue)', color: '#3b82f6', x: 335, y: 260, w: 45, h: 45, count: 0 },
  ];

  // Simulator State
  let caps = [];
  let sortedCaps = [];
  let gantry = {
    x: 0,
    y: 0,
    z: 'RAISED', // 'RAISED' or 'LOWERED'
    zHeight: 0, // 0 = raised, 1 = lowered
    gripper: 'OPEN', // 'OPEN' or 'CLOSED'
    gripperWidth: 36, // 36mm open, 26mm closed
    heldCap: null,
    targetX: 0,
    targetY: 0,
    speed: 120, // mm/s
    isHomed: true,
  };

  let simState = 'IDLE'; // IDLE, SCANNING, MOVING_TO_CAP, LOWERING, GRIPPING, LIFTING, MOVING_TO_BIN, LOWERING_BIN, RELEASING, RETRACTING, HOMING, COMM_RECOVERY, HALTED
  let planQueue = [];
  let currentTarget = null;
  let animTimer = 0;
  let cycleStartTime = 0;
  let capCycleStart = 0;
  let lastFrameTime = performance.now();
  let simSpeedMultiplier = 1;
  let commsLost = false;
  let selectedColor = 'red';

  // Seed default caps
  resetToDefaultCaps();

  function resetToDefaultCaps() {
    caps = [
      { id: 1, color: 'red', x: 45, y: 45, cell: 'C1', status: 'pending' },
      { id: 2, color: 'yellow', x: 195, y: 75, cell: 'C2', status: 'pending' },
      { id: 3, color: 'green', x: 75, y: 225, cell: 'C3', status: 'pending' },
      { id: 4, color: 'blue', x: 225, y: 195, cell: 'C4', status: 'pending' },
      { id: 5, color: 'red', x: 135, y: 165, cell: 'C5', status: 'pending' },
      { id: 6, color: 'yellow', x: 255, y: 255, cell: 'C6', status: 'pending' },
    ];
    BINS.forEach(b => b.count = 0);
    sortedCaps = [];
    gantry.heldCap = null;
    gantry.x = 0;
    gantry.y = 0;
    gantry.z = 'RAISED';
    gantry.zHeight = 0;
    gantry.gripper = 'OPEN';
    simState = 'IDLE';
    planQueue = [];
    commsLost = false;
    updateTelemetryUI();
  }

  // Click on canvas to add caps or pick color
  canvas.addEventListener('click', (e) => {
    if (simState !== 'IDLE') return;
    const rect = canvas.getBoundingClientRect();
    const mouseX = (e.clientX - rect.left) * (canvas.width / rect.width);
    const mouseY = (e.clientY - rect.top) * (canvas.height / rect.height);

    // Convert pixel to mm
    const boardX = (mouseX - PADDING_LEFT) / SCALE;
    const boardY = (mouseY - PADDING_TOP) / SCALE;

    if (boardX >= 0 && boardX <= BOARD_SIZE_MM && boardY >= 0 && boardY <= BOARD_SIZE_MM) {
      // Snap to cell center
      const cellCol = Math.floor(boardX / CELL_SIZE_MM);
      const cellRow = Math.floor(boardY / CELL_SIZE_MM);
      const snapX = cellCol * CELL_SIZE_MM + CELL_SIZE_MM / 2;
      const snapY = cellRow * CELL_SIZE_MM + CELL_SIZE_MM / 2;

      // Check if already occupied
      const existingIdx = caps.findIndex(c => Math.hypot(c.x - snapX, c.y - snapY) < 15);
      if (existingIdx >= 0) {
        caps.splice(existingIdx, 1);
      } else {
        caps.push({
          id: Date.now() + Math.random(),
          color: selectedColor,
          x: snapX,
          y: snapY,
          cell: `(${cellCol},${cellRow})`,
          status: 'pending'
        });
      }
      updateTelemetryUI();
    }
  });

  // Buttons
  document.getElementById('btn-start-sort')?.addEventListener('click', startSorting);
  document.getElementById('btn-reset-sim')?.addEventListener('click', () => {
    resetToDefaultCaps();
  });
  document.getElementById('btn-preset-scatter')?.addEventListener('click', () => {
    caps = [];
    const colors = ['red', 'yellow', 'green', 'blue'];
    for (let i = 0; i < 8; i++) {
      const col = Math.floor(Math.random() * 10);
      const row = Math.floor(Math.random() * 10);
      const snapX = col * CELL_SIZE_MM + CELL_SIZE_MM / 2;
      const snapY = row * CELL_SIZE_MM + CELL_SIZE_MM / 2;
      if (!caps.some(c => c.x === snapX && c.y === snapY)) {
        caps.push({
          id: Date.now() + i,
          color: colors[i % 4],
          x: snapX,
          y: snapY,
          cell: `(${col},${row})`,
          status: 'pending'
        });
      }
    }
    BINS.forEach(b => b.count = 0);
    sortedCaps = [];
    simState = 'IDLE';
    updateTelemetryUI();
  });

  document.getElementById('btn-clear-sim')?.addEventListener('click', () => {
    caps = [];
    BINS.forEach(b => b.count = 0);
    sortedCaps = [];
    simState = 'IDLE';
    updateTelemetryUI();
  });

  document.getElementById('btn-sim-speed')?.addEventListener('click', (e) => {
    if (simSpeedMultiplier === 1) {
      simSpeedMultiplier = 2;
      e.target.textContent = 'Speed: 2x';
    } else if (simSpeedMultiplier === 2) {
      simSpeedMultiplier = 4;
      e.target.textContent = 'Speed: 4x';
    } else {
      simSpeedMultiplier = 1;
      e.target.textContent = 'Speed: 1x';
    }
  });

  document.getElementById('btn-sim-comms-cut')?.addEventListener('click', () => {
    if (simState === 'IDLE') return;
    commsLost = true;
    showNotification('UART Link Interrupted! Mega executing localized safe completion sequence...', 'warning');
  });

  // Color picker radios
  document.querySelectorAll('.cap-color-picker').forEach(el => {
    el.addEventListener('click', () => {
      document.querySelectorAll('.cap-color-picker').forEach(b => b.classList.remove('ring-2', 'ring-white'));
      el.classList.add('ring-2', 'ring-white');
      selectedColor = el.dataset.color;
    });
  });

  function startSorting() {
    if (caps.length === 0) {
      showNotification('Please place caps on the grid first!', 'info');
      return;
    }
    if (simState !== 'IDLE') return;

    simState = 'SCANNING';
    animTimer = 0;
    cycleStartTime = performance.now();
    capCycleStart = cycleStartTime;
    commsLost = false;

    // Plan sequence: Nearest-neighbor heuristic for shortest travel
    let remaining = [...caps];
    planQueue = [];
    let curX = gantry.x;
    let curY = gantry.y;

    while (remaining.length > 0) {
      // Find closest cap
      remaining.sort((a, b) => Math.hypot(a.x - curX, a.y - curY) - Math.hypot(b.x - curX, b.y - curY));
      const nextCap = remaining.shift();
      planQueue.push(nextCap);
      const bin = BINS.find(b => b.id === nextCap.color);
      curX = bin.x;
      curY = bin.y;
    }

    showNotification(`Vision computed ${planQueue.length} cap targets. Sorting plan dispatched to Mega!`, 'success');
  }

  // Animation Loop
  function tick(timestamp) {
    const dt = Math.min((timestamp - lastFrameTime) / 1000, 0.1) * simSpeedMultiplier;
    lastFrameTime = timestamp;

    updateSimulation(dt);
    renderSimulation();

    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  function updateSimulation(dt) {
    if (simState === 'IDLE') return;

    if (simState === 'SCANNING') {
      animTimer += dt;
      if (animTimer > 0.8) {
        simState = 'NEXT_CAP';
      }
      return;
    }

    if (simState === 'NEXT_CAP') {
      if (commsLost && !gantry.heldCap) {
        // Safe halt on comms lost if no cap is currently in flight
        simState = 'SAFE_HALTED';
        showNotification('FAIL-SAFE TRIGGERED: System safely parked at standby coordinates. Ready for recovery.', 'error');
        updateTelemetryUI();
        return;
      }

      if (planQueue.length === 0) {
        // All caps sorted!
        simState = 'PARKING';
        gantry.targetX = 0;
        gantry.targetY = 0;
        return;
      }

      currentTarget = planQueue.shift();
      gantry.targetX = currentTarget.x;
      gantry.targetY = currentTarget.y;
      simState = 'MOVING_TO_CAP';
      capCycleStart = performance.now();
      return;
    }

    if (simState === 'MOVING_TO_CAP') {
      const reached = moveGantryTowards(gantry.targetX, gantry.targetY, dt);
      if (reached) {
        simState = 'LOWERING_Z';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'LOWERING_Z') {
      animTimer += dt * 3.5;
      gantry.zHeight = Math.min(1, animTimer);
      if (gantry.zHeight >= 1) {
        gantry.z = 'LOWERED';
        simState = 'GRIPPING';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'GRIPPING') {
      animTimer += dt * 4;
      gantry.gripperWidth = 36 - 10 * Math.min(1, animTimer);
      if (animTimer >= 1) {
        gantry.gripper = 'CLOSED';
        gantry.heldCap = currentTarget;
        // Remove from pending caps
        caps = caps.filter(c => c.id !== currentTarget.id);
        simState = 'LIFTING_Z';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'LIFTING_Z') {
      animTimer += dt * 3.5;
      gantry.zHeight = Math.max(0, 1 - animTimer);
      if (gantry.zHeight <= 0) {
        gantry.z = 'RAISED';
        // Now head to matching bin
        const bin = BINS.find(b => b.id === gantry.heldCap.color);
        gantry.targetX = bin.x + bin.w / 2;
        gantry.targetY = bin.y + bin.h / 2;
        simState = 'MOVING_TO_BIN';
      }
      return;
    }

    if (simState === 'MOVING_TO_BIN') {
      const reached = moveGantryTowards(gantry.targetX, gantry.targetY, dt);
      if (reached) {
        simState = 'LOWERING_BIN';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'LOWERING_BIN') {
      animTimer += dt * 3.5;
      gantry.zHeight = Math.min(0.6, animTimer);
      if (animTimer >= 0.6) {
        simState = 'RELEASING';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'RELEASING') {
      animTimer += dt * 4;
      gantry.gripperWidth = 26 + 10 * Math.min(1, animTimer);
      if (animTimer >= 1) {
        gantry.gripper = 'OPEN';
        // Deposit into bin
        const bin = BINS.find(b => b.id === gantry.heldCap.color);
        bin.count++;
        sortedCaps.push({ ...gantry.heldCap, x: bin.x + 10 + (bin.count * 4) % 25, y: bin.y + 10 + (bin.count * 6) % 25 });
        gantry.heldCap = null;
        simState = 'RETRACTING_BIN';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'RETRACTING_BIN') {
      animTimer += dt * 3.5;
      gantry.zHeight = Math.max(0, 0.6 - animTimer);
      if (gantry.zHeight <= 0) {
        gantry.z = 'RAISED';
        updateTelemetryUI();

        if (commsLost) {
          // Finish current cap safe recovery protocol:
          simState = 'SAFE_HALTED';
          showNotification('FAIL-SAFE SUCCESS: In-flight cap safely sorted into bin. Machine safely halted.', 'warning');
        } else {
          simState = 'NEXT_CAP';
        }
      }
      return;
    }

    if (simState === 'PARKING') {
      const reached = moveGantryTowards(0, 0, dt);
      if (reached) {
        simState = 'IDLE';
        showNotification('Sorting batch completed successfully! All caps sorted.', 'success');
        updateTelemetryUI();
      }
      return;
    }
  }

  function moveGantryTowards(tx, ty, dt) {
    const dx = tx - gantry.x;
    const dy = ty - gantry.y;
    const dist = Math.hypot(dx, dy);
    const step = gantry.speed * dt;

    if (dist <= step) {
      gantry.x = tx;
      gantry.y = ty;
      return true;
    } else {
      gantry.x += (dx / dist) * step;
      gantry.y += (dy / dist) * step;
      return false;
    }
  }

  function renderSimulation() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Draw outer boundary and 300x300mm sorting board
    const boardPx = BOARD_SIZE_MM * SCALE;
    ctx.fillStyle = '#0f1422';
    ctx.fillRect(PADDING_LEFT, PADDING_TOP, boardPx, boardPx);

    // Grid lines (10x10)
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let i = 0; i <= GRID_DIVS; i++) {
      const pos = PADDING_LEFT + i * (CELL_SIZE_MM * SCALE);
      ctx.beginPath();
      ctx.moveTo(pos, PADDING_TOP);
      ctx.lineTo(pos, PADDING_TOP + boardPx);
      ctx.stroke();

      const ypos = PADDING_TOP + i * (CELL_SIZE_MM * SCALE);
      ctx.beginPath();
      ctx.moveTo(PADDING_LEFT, ypos);
      ctx.lineTo(PADDING_LEFT + boardPx, ypos);
      ctx.stroke();
    }

    // Grid Labels (A-J, 1-10)
    ctx.fillStyle = '#64748b';
    ctx.font = '10px JetBrains Mono';
    for (let i = 0; i < GRID_DIVS; i++) {
      const x = PADDING_LEFT + (i + 0.5) * (CELL_SIZE_MM * SCALE);
      ctx.fillText(String.fromCharCode(65 + i), x - 4, PADDING_TOP - 8);
      const y = PADDING_TOP + (i + 0.5) * (CELL_SIZE_MM * SCALE);
      ctx.fillText((i + 1).toString(), PADDING_LEFT - 18, y + 4);
    }

    // Board outline
    ctx.strokeStyle = '#ff6b00';
    ctx.lineWidth = 2;
    ctx.strokeRect(PADDING_LEFT, PADDING_TOP, boardPx, boardPx);

    // Corner calibration markers
    const markerSize = 10;
    const corners = [
      [PADDING_LEFT, PADDING_TOP],
      [PADDING_LEFT + boardPx, PADDING_TOP],
      [PADDING_LEFT, PADDING_TOP + boardPx],
      [PADDING_LEFT + boardPx, PADDING_TOP + boardPx]
    ];
    corners.forEach(([cx, cy]) => {
      ctx.fillStyle = '#ff6b00';
      ctx.fillRect(cx - markerSize / 2, cy - markerSize / 2, markerSize, markerSize);
    });

    // 2. Draw 4 Sorting Bins
    BINS.forEach((bin, idx) => {
      const bx = PADDING_LEFT + bin.x * SCALE;
      const by = PADDING_TOP + bin.y * SCALE;
      const bw = bin.w * SCALE;
      const bh = bin.h * SCALE;

      // Bin container box
      ctx.fillStyle = '#161d2d';
      ctx.fillRect(bx, by, bw, bh);
      ctx.strokeStyle = bin.color;
      ctx.lineWidth = 2;
      ctx.strokeRect(bx, by, bw, bh);

      // Bin label
      ctx.fillStyle = bin.color;
      ctx.font = 'bold 11px Outfit, sans-serif';
      ctx.fillText(bin.name, bx, by - 6);

      // Count badge
      ctx.fillStyle = '#cbd5e1';
      ctx.font = '12px JetBrains Mono';
      ctx.fillText(`Qty: ${bin.count}`, bx + 8, by + bh / 2 + 4);
    });

    // 3. Draw Pending Caps on Board
    caps.forEach(cap => {
      const cx = PADDING_LEFT + cap.x * SCALE;
      const cy = PADDING_TOP + cap.y * SCALE;
      const cr = CAP_RADIUS_MM * SCALE;

      // Outer cap body
      ctx.beginPath();
      ctx.arc(cx, cy, cr, 0, Math.PI * 2);
      ctx.fillStyle = getCapHex(cap.color);
      ctx.fill();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Cap inner ridge (ribbed texture)
      ctx.beginPath();
      ctx.arc(cx, cy, cr * 0.7, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.3)';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Coordinate crosshair
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx - 4, cy); ctx.lineTo(cx + 4, cy);
      ctx.moveTo(cx, cy - 4); ctx.lineTo(cx, cy + 4);
      ctx.stroke();
    });

    // 4. Draw Gantry Rails and Moving Carriage
    const gantryPxX = PADDING_LEFT + gantry.x * SCALE;
    const gantryPxY = PADDING_TOP + gantry.y * SCALE;

    // Y-Axis Gantry Bridge Beam (horizontal beam across workspace)
    ctx.fillStyle = 'rgba(30, 41, 59, 0.85)';
    ctx.fillRect(PADDING_LEFT - 10, gantryPxY - 8, boardPx + 100, 16);
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1;
    ctx.strokeRect(PADDING_LEFT - 10, gantryPxY - 8, boardPx + 100, 16);

    // X-Axis Carriage Block
    ctx.fillStyle = '#ff6b00';
    ctx.fillRect(gantryPxX - 16, gantryPxY - 16, 32, 32);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(gantryPxX - 16, gantryPxY - 16, 32, 32);

    // Z-Linkage Indicator
    if (gantry.zHeight > 0.05) {
      ctx.fillStyle = `rgba(0, 210, 255, ${gantry.zHeight * 0.6})`;
      ctx.beginPath();
      ctx.arc(gantryPxX, gantryPxY, 24 * gantry.zHeight, 0, Math.PI * 2);
      ctx.fill();
    }

    // 2-Finger Gripper Jaws
    const jawHalfGap = (gantry.gripperWidth * SCALE) / 2;
    ctx.fillStyle = '#f8fafc';
    // Left finger
    ctx.fillRect(gantryPxX - jawHalfGap - 5, gantryPxY - 14, 5, 28);
    // Right finger
    ctx.fillRect(gantryPxX + jawHalfGap, gantryPxY - 14, 5, 28);

    // If a cap is held by gantry
    if (gantry.heldCap) {
      const cr = CAP_RADIUS_MM * SCALE;
      ctx.beginPath();
      ctx.arc(gantryPxX, gantryPxY, cr, 0, Math.PI * 2);
      ctx.fillStyle = getCapHex(gantry.heldCap.color);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Camera FOV outline
    if (simState === 'SCANNING') {
      ctx.strokeStyle = '#00d2ff';
      ctx.lineWidth = 2;
      ctx.setLineDash([8, 4]);
      ctx.strokeRect(PADDING_LEFT + 5, PADDING_TOP + 5, boardPx - 10, boardPx - 10);
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(0, 210, 255, 0.08)';
      ctx.fillRect(PADDING_LEFT, PADDING_TOP, boardPx, boardPx);

      ctx.fillStyle = '#00d2ff';
      ctx.font = 'bold 14px Outfit, sans-serif';
      ctx.fillText('CAMERA ACQUISITION & HSV SEGMENTATION ACTIVE...', PADDING_LEFT + 40, PADDING_TOP + boardPx / 2);
    }

    // Telemetry text overlay
    updateCoordinatesUI(gantry.x, gantry.y, gantry.z, gantry.gripper, simState);
  }

  function getCapHex(color) {
    switch (color) {
      case 'red': return '#ef4444';
      case 'yellow': return '#eab308';
      case 'green': return '#10b981';
      case 'blue': return '#3b82f6';
      default: return '#94a3b8';
    }
  }

  function updateCoordinatesUI(x, y, z, gripper, state) {
    const elX = document.getElementById('telemetry-x');
    const elY = document.getElementById('telemetry-y');
    const elZ = document.getElementById('telemetry-z');
    const elGrip = document.getElementById('telemetry-gripper');
    const elState = document.getElementById('telemetry-state');

    if (elX) elX.textContent = `${Math.round(x)} mm`;
    if (elY) elY.textContent = `${Math.round(y)} mm`;
    if (elZ) elZ.textContent = z;
    if (elGrip) elGrip.textContent = gripper;
    if (elState) {
      elState.textContent = state.replace(/_/g, ' ');
      elState.className = state === 'SAFE_HALTED' ? 'text-red-400 font-bold' : (state === 'IDLE' ? 'text-slate-400 font-mono' : 'text-emerald-400 font-bold');
    }
  }

  function updateTelemetryUI() {
    const elTotal = document.getElementById('telemetry-total-caps');
    const elSorted = document.getElementById('telemetry-sorted-count');
    const elPending = document.getElementById('telemetry-pending-count');

    if (elTotal) elTotal.textContent = (caps.length + sortedCaps.length).toString();
    if (elSorted) elSorted.textContent = sortedCaps.length.toString();
    if (elPending) elPending.textContent = caps.length.toString();
  }
}

/* ==========================================================================
   2. Master Infographic Poster Modal & Pan/Zoom
   ========================================================================== */
function initPosterModal() {
  const modal = document.getElementById('poster-modal');
  const openBtns = document.querySelectorAll('.btn-open-poster');
  const closeBtn = document.getElementById('btn-close-poster');
  const posterImg = document.getElementById('modal-poster-img');
  const zoomIn = document.getElementById('btn-zoom-in');
  const zoomOut = document.getElementById('btn-zoom-out');
  const zoomReset = document.getElementById('btn-zoom-reset');

  if (!modal || !posterImg) return;

  let scale = 1;
  let isPanning = false;
  let startX = 0, startY = 0;
  let translateX = 0, translateY = 0;

  function updateTransform() {
    posterImg.style.transform = `translate(${translateX}px, ${translateY}px) scale(${scale})`;
  }

  openBtns.forEach(btn => {
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      modal.classList.add('open');
      document.body.style.overflow = 'hidden';
      scale = 1;
      translateX = 0;
      translateY = 0;
      updateTransform();
    });
  });

  function closeModal() {
    modal.classList.remove('open');
    document.body.style.overflow = '';
  }

  closeBtn?.addEventListener('click', closeModal);
  modal.addEventListener('click', (e) => {
    if (e.target === modal || e.target.classList.contains('poster-viewport')) {
      closeModal();
    }
  });

  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && modal.classList.contains('open')) {
      closeModal();
    }
  });

  zoomIn?.addEventListener('click', () => {
    scale = Math.min(3.5, scale + 0.3);
    updateTransform();
  });

  zoomOut?.addEventListener('click', () => {
    scale = Math.max(0.6, scale - 0.3);
    updateTransform();
  });

  zoomReset?.addEventListener('click', () => {
    scale = 1;
    translateX = 0;
    translateY = 0;
    updateTransform();
  });

  // Pan interaction
  const viewport = document.querySelector('.poster-viewport');
  if (viewport) {
    viewport.addEventListener('mousedown', (e) => {
      if (scale <= 1) return;
      isPanning = true;
      startX = e.clientX - translateX;
      startY = e.clientY - translateY;
    });

    window.addEventListener('mousemove', (e) => {
      if (!isPanning) return;
      translateX = e.clientX - startX;
      translateY = e.clientY - startY;
      updateTransform();
    });

    window.addEventListener('mouseup', () => {
      isPanning = false;
    });

    // Wheel zoom
    viewport.addEventListener('wheel', (e) => {
      e.preventDefault();
      const delta = e.deltaY * -0.0015;
      scale = Math.min(Math.max(0.6, scale + delta), 3.5);
      updateTransform();
    }, { passive: false });
  }
}

/* ==========================================================================
   3. Vision Pipeline Step-by-Step Demonstrator
   ========================================================================== */
function initVisionPipeline() {
  const stepBtns = document.querySelectorAll('.vision-step-btn');
  const stepPanels = document.querySelectorAll('.vision-step-panel');

  stepBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      const targetStep = btn.dataset.step;

      stepBtns.forEach(b => {
        b.classList.remove('bg-orange-500/20', 'text-orange-400', 'border-orange-500/40');
        b.classList.add('bg-slate-800/40', 'text-slate-400', 'border-slate-700/40');
      });
      btn.classList.add('bg-orange-500/20', 'text-orange-400', 'border-orange-500/40');
      btn.classList.remove('bg-slate-800/40', 'text-slate-400', 'border-slate-700/40');

      stepPanels.forEach(panel => {
        if (panel.id === `vision-panel-${targetStep}`) {
          panel.classList.remove('hidden');
        } else {
          panel.classList.add('hidden');
        }
      });
    });
  });
}

/* ==========================================================================
   4. Pinout Matrix Search & Controller Filtering
   ========================================================================== */
function initPinoutTable() {
  const searchInput = document.getElementById('pinout-search');
  const filterBtns = document.querySelectorAll('.pinout-filter-btn');
  const rows = document.querySelectorAll('#pinout-table-body tr');

  function filterTable() {
    const query = (searchInput?.value || '').toLowerCase();
    const activeFilter = document.querySelector('.pinout-filter-btn.active')?.dataset.filter || 'all';

    rows.forEach(row => {
      const text = row.textContent.toLowerCase();
      const controller = row.dataset.controller || '';

      const matchesSearch = text.includes(query);
      const matchesController = (activeFilter === 'all') || (controller === activeFilter);

      if (matchesSearch && matchesController) {
        row.style.display = '';
      } else {
        row.style.display = 'none';
      }
    });
  }

  searchInput?.addEventListener('input', filterTable);

  filterBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      filterBtns.forEach(b => b.classList.remove('active', 'bg-orange-500/20', 'text-orange-400', 'border-orange-500/40'));
      btn.classList.add('active', 'bg-orange-500/20', 'text-orange-400', 'border-orange-500/40');
      filterTable();
    });
  });
}

/* ==========================================================================
   5. Bill of Materials (BOM) Interactive Table
   ========================================================================== */
function initBOMTable() {
  const searchInput = document.getElementById('bom-search');
  const filterBtns = document.querySelectorAll('.bom-filter-btn');
  const rows = document.querySelectorAll('#bom-table-body tr');

  function filterBOM() {
    const query = (searchInput?.value || '').toLowerCase();
    const activeFilter = document.querySelector('.bom-filter-btn.active')?.dataset.filter || 'all';

    rows.forEach(row => {
      const text = row.textContent.toLowerCase();
      const status = row.dataset.status || '';

      const matchesSearch = text.includes(query);
      const matchesStatus = (activeFilter === 'all') || (status === activeFilter);

      if (matchesSearch && matchesStatus) {
        row.style.display = '';
      } else {
        row.style.display = 'none';
      }
    });
  }

  searchInput?.addEventListener('input', filterBOM);

  filterBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      filterBtns.forEach(b => b.classList.remove('active', 'bg-orange-500/20', 'text-orange-400', 'border-orange-500/40'));
      btn.classList.add('active', 'bg-orange-500/20', 'text-orange-400', 'border-orange-500/40');
      filterBOM();
    });
  });
}

/* ==========================================================================
   6. Firmware Code Vault Tabs & Copy
   ========================================================================== */
function initCodeVault() {
  const tabs = document.querySelectorAll('.code-tab-btn');
  const blocks = document.querySelectorAll('.code-block');

  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      const targetId = tab.dataset.code;

      tabs.forEach(t => {
        t.classList.remove('text-orange-400', 'border-orange-500', 'bg-orange-500/10');
        t.classList.add('text-slate-400', 'border-transparent');
      });
      tab.classList.add('text-orange-400', 'border-orange-500', 'bg-orange-500/10');
      tab.classList.remove('text-slate-400', 'border-transparent');

      blocks.forEach(block => {
        if (block.id === `code-${targetId}`) {
          block.classList.remove('hidden');
        } else {
          block.classList.add('hidden');
        }
      });
    });
  });

  // Copy code buttons
  document.querySelectorAll('.btn-copy-code').forEach(btn => {
    btn.addEventListener('click', () => {
      const targetBlockId = btn.dataset.target;
      const codeEl = document.querySelector(`#${targetBlockId} code`);
      if (codeEl) {
        navigator.clipboard.writeText(codeEl.innerText).then(() => {
          showNotification('Firmware code copied to clipboard!', 'success');
          const original = btn.innerHTML;
          btn.innerHTML = `<span class="text-emerald-400 font-bold">✓ Copied</span>`;
          setTimeout(() => btn.innerHTML = original, 2000);
        });
      }
    });
  });
}

/* ==========================================================================
   7. UI Notification Toasts & Smooth Scrolling
   ========================================================================== */
function showNotification(msg, type = 'info') {
  const toast = document.createElement('div');
  const borderCol = type === 'success' ? 'border-emerald-500' : (type === 'warning' ? 'border-amber-500' : (type === 'error' ? 'border-red-500' : 'border-cyan-500'));
  const textCol = type === 'success' ? 'text-emerald-300' : (type === 'warning' ? 'text-amber-300' : (type === 'error' ? 'text-red-300' : 'text-cyan-300'));

  toast.className = `fixed bottom-6 right-6 z-50 flex items-center gap-3 px-4 py-3 rounded-lg bg-slate-900/95 border ${borderCol} shadow-2xl backdrop-blur-md transition-all duration-300 transform translate-y-4 opacity-0`;
  toast.innerHTML = `
    <div class="w-2.5 h-2.5 rounded-full ${type === 'success' ? 'bg-emerald-400' : (type === 'warning' ? 'bg-amber-400' : (type === 'error' ? 'bg-red-400' : 'bg-cyan-400'))}"></div>
    <span class="text-sm font-medium ${textCol}">${msg}</span>
  `;

  document.body.appendChild(toast);
  requestAnimationFrame(() => {
    toast.classList.remove('translate-y-4', 'opacity-0');
  });

  setTimeout(() => {
    toast.classList.add('translate-y-4', 'opacity-0');
    setTimeout(() => toast.remove(), 300);
  }, 4000);
}

function initSmoothScroll() {
  document.querySelectorAll('a[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', function (e) {
      const target = document.querySelector(this.getAttribute('href'));
      if (target) {
        e.preventDefault();
        target.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    });
  });
}
