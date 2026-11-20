/**
 * Autonomous Bottle-Cap Sorting Machine (CapSorter)
 * Interactive Application, Smooth Motion & HiDPI Simulation Engine
 */

document.addEventListener('DOMContentLoaded', () => {
  if (window.lucide) {
    window.lucide.createIcons();
  }

  initChromePolish();
  initMobileMenu();
  initScrollReveal();
  initSimulator();
  initPosterModal();
  initVisionPipeline();
  initPinoutTable();
  initBOMTable();
  initCodeVault();
  initEngineeringDrawer();
  initEngineeringScrollSpy();
  initScrollSpyNav();
  initSmoothScroll();
});

function isV2Theme() {
  return document.body?.dataset?.version === 'v2';
}

/* ==========================================================================
   0. Chrome Polish: Reading Progress Bar, Elevated Header & Back-to-Top
   ========================================================================== */
function initChromePolish() {
  const header = document.querySelector('header');
  if (header) header.classList.add('site-header');

  // Inject top scroll progress bar
  const progressBar = document.createElement('div');
  progressBar.className = 'scroll-progress-bar';
  progressBar.setAttribute('aria-hidden', 'true');
  document.body.prepend(progressBar);

  // Inject floating Back-to-Top button
  const backTopBtn = document.createElement('button');
  backTopBtn.type = 'button';
  backTopBtn.className = 'back-to-top-btn';
  backTopBtn.setAttribute('aria-label', 'Scroll back to top');
  backTopBtn.title = 'Back to top';
  backTopBtn.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"><path d="m18 15-6-6-6 6"/></svg>`;
  backTopBtn.addEventListener('click', () => {
    window.scrollTo({ top: 0, behavior: 'smooth' });
  });
  document.body.appendChild(backTopBtn);

  let ticking = false;
  function updateScrollChrome() {
    ticking = false;
    const scrollY = window.scrollY || window.pageYOffset;
    const maxScroll = Math.max(1, document.documentElement.scrollHeight - window.innerHeight);
    const ratio = Math.min(1, Math.max(0, scrollY / maxScroll));

    progressBar.style.transform = `scaleX(${ratio})`;
    if (header) header.classList.toggle('is-scrolled', scrollY > 14);
    backTopBtn.classList.toggle('visible', scrollY > 520);
  }

  window.addEventListener('scroll', () => {
    if (!ticking) {
      ticking = true;
      requestAnimationFrame(updateScrollChrome);
    }
  }, { passive: true });
  window.addEventListener('resize', updateScrollChrome, { passive: true });
  updateScrollChrome();
}

/* ==========================================================================
   0.5 Mobile Slide-Over Navigation Drawer
   ========================================================================== */
function initMobileMenu() {
  const openBtn = document.getElementById('btn-mobile-menu');
  const closeBtn = document.getElementById('btn-mobile-menu-close');
  const drawer = document.getElementById('mobile-nav-drawer');
  const backdrop = document.getElementById('mobile-nav-backdrop');

  if (!openBtn || !drawer || !backdrop) return;

  function setMenuOpen(open) {
    drawer.classList.toggle('open', open);
    backdrop.classList.toggle('open', open);
    drawer.setAttribute('aria-hidden', String(!open));
    openBtn.setAttribute('aria-expanded', String(open));
    document.body.style.overflow = open ? 'hidden' : '';
    if (open) {
      closeBtn?.focus({ preventScroll: true });
    }
  }

  openBtn.addEventListener('click', () => setMenuOpen(true));
  closeBtn?.addEventListener('click', () => setMenuOpen(false));
  backdrop.addEventListener('click', () => setMenuOpen(false));

  drawer.querySelectorAll('a, button.mobile-nav-action').forEach(el => {
    el.addEventListener('click', () => setMenuOpen(false));
  });

  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && drawer.classList.contains('open')) {
      setMenuOpen(false);
      openBtn.focus({ preventScroll: true });
    }
  });
}

/* ==========================================================================
   0.75 Subtle GPU Scroll-Reveal System
   ========================================================================== */
function initScrollReveal() {
  if (
    !('IntersectionObserver' in window) ||
    window.matchMedia('(prefers-reduced-motion: reduce)').matches
  ) {
    return;
  }

  // Only reveal cards inside non-hero sections that are outside the collapsed drawer
  const targets = Array.from(document.querySelectorAll(
    'section[id] .tech-card, section[id] .tech-panel'
  )).filter(el => !el.closest('#engineering-drawer'));

  if (targets.length === 0) return;

  document.documentElement.classList.add('js-reveal-ready');

  targets.forEach((el, idx) => {
    el.classList.add('reveal-item');
    // Subtle sibling stagger within grid containers
    const parent = el.parentElement;
    if (parent && parent.classList.contains('grid')) {
      const siblingIndex = Array.from(parent.children).indexOf(el);
      const delayMs = Math.min(siblingIndex * 40, 160);
      el.style.transitionDelay = `${delayMs}ms`;
    } else if (idx < 2) {
      el.style.transitionDelay = '0ms';
    }
  });

  const observer = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        entry.target.classList.add('is-visible');
        observer.unobserve(entry.target);
        // Clear delay after reveal so hover transitions remain snappy
        setTimeout(() => {
          entry.target.style.transitionDelay = '';
        }, 600);
      }
    });
  }, {
    threshold: 0.08,
    rootMargin: '0px 0px -30px 0px'
  });

  targets.forEach(el => observer.observe(el));
}

/* ==========================================================================
   1. Gantry Pick & Place 2D Simulator (HiDPI, Eased Motion, Zero Idle Drain)
   ========================================================================== */
function initSimulator() {
  const canvas = document.getElementById('simulation-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const isV2 = isV2Theme();
  const accentPrimary = isV2 ? '#00d2ff' : '#ff6b00';
  const accentGlow = isV2 ? 'rgba(0, 210, 255, 0.25)' : 'rgba(255, 107, 0, 0.25)';

  // Logical coordinate space (580 x 500) scaled by DPR for Retina crispness
  const LOGICAL_W = 580;
  const LOGICAL_H = 500;
  const DPR = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = LOGICAL_W * DPR;
  canvas.height = LOGICAL_H * DPR;
  ctx.scale(DPR, DPR);

  // Physical specs & centered layout geometry
  const BOARD_SIZE_MM = 300;
  const GRID_DIVS = 10;
  const CELL_SIZE_MM = 30;
  const CAP_RADIUS_MM = 14.5;

  const PADDING_LEFT = 48;
  const PADDING_TOP = 50;
  const GRID_PIXELS = 360;
  const SCALE = GRID_PIXELS / BOARD_SIZE_MM; // 1.2 px per mm

  // 4 Output Bins cleanly balanced on the right of the 300x300mm grid
  const BINS = [
    { id: 'red', name: 'BIN 1 • RED', color: '#ef4444', bg: 'rgba(239,68,68,0.10)', headerBg: 'rgba(239,68,68,0.22)', x: 326, y: 12, w: 66, h: 64, count: 0 },
    { id: 'yellow', name: 'BIN 2 • YLW', color: '#eab308', bg: 'rgba(234,179,8,0.10)', headerBg: 'rgba(234,179,8,0.22)', x: 326, y: 88, w: 66, h: 64, count: 0 },
    { id: 'green', name: 'BIN 3 • GRN', color: '#10b981', bg: 'rgba(16,185,129,0.10)', headerBg: 'rgba(16,185,129,0.22)', x: 326, y: 164, w: 66, h: 64, count: 0 },
    { id: 'blue', name: 'BIN 4 • BLU', color: '#3b82f6', bg: 'rgba(59,130,246,0.10)', headerBg: 'rgba(59,130,246,0.22)', x: 326, y: 240, w: 66, h: 64, count: 0 },
  ];

  // Cache Telemetry DOM nodes (zero per-frame getElementById thrashing)
  const uiEls = {
    x: document.getElementById('telemetry-x'),
    y: document.getElementById('telemetry-y'),
    z: document.getElementById('telemetry-z'),
    grip: document.getElementById('telemetry-gripper'),
    state: document.getElementById('telemetry-state'),
    total: document.getElementById('telemetry-total-caps'),
    sorted: document.getElementById('telemetry-sorted-count'),
    pending: document.getElementById('telemetry-pending-count'),
    startBtn: document.getElementById('btn-start-sort'),
    commsBtn: document.getElementById('btn-sim-comms-cut'),
  };
  const defaultStartBtnHTML = uiEls.startBtn?.innerHTML || '';
  const defaultStartBtnClass = uiEls.startBtn?.className || '';
  const lastTelemetry = { x: null, y: null, z: null, grip: null, state: null };

  // Simulator State
  let caps = [];
  let sortedCaps = [];
  let hoverCell = null; // { col, row, snapX, snapY, occupied }
  let gantry = {
    x: 0,
    y: 0,
    startX: 0,
    startY: 0,
    targetX: 0,
    targetY: 0,
    z: 'RAISED',
    zHeight: 0,
    gripper: 'OPEN',
    gripperWidth: 36,
    heldCap: null,
    speed: 90, // mm/s nominal display speed (70-90 mm/s smooth profile)
  };

  let simState = 'IDLE';
  let planQueue = [];
  let currentTarget = null;
  let animTimer = 0;
  let scanSweepY = 0;
  let simElapsedSec = 0;
  let lastFrameTime = performance.now();
  let simSpeedMultiplier = 1;
  let commsLost = false;
  let selectedColor = 'red';
  let rafId = null;

  function ensureLoopRunning() {
    if (rafId !== null) return;
    lastFrameTime = performance.now();
    rafId = requestAnimationFrame(tick);
  }

  function renderOnce() {
    renderSimulation();
    syncCoordinatesUI();
  }

  function resetGantryPose() {
    gantry.x = 0;
    gantry.y = 0;
    gantry.startX = 0;
    gantry.startY = 0;
    gantry.targetX = 0;
    gantry.targetY = 0;
    gantry.z = 'RAISED';
    gantry.zHeight = 0;
    gantry.gripper = 'OPEN';
    gantry.gripperWidth = 36;
    gantry.heldCap = null;
    currentTarget = null;
    planQueue = [];
    commsLost = false;
    simElapsedSec = 0;
    simState = 'IDLE';
    updateControlButtonsState();
  }

  function resetToDefaultCaps() {
    caps = [
      { id: 1, color: 'red', x: 45, y: 45, status: 'pending' },
      { id: 2, color: 'yellow', x: 195, y: 75, status: 'pending' },
      { id: 3, color: 'green', x: 75, y: 225, status: 'pending' },
      { id: 4, color: 'blue', x: 225, y: 195, status: 'pending' },
      { id: 5, color: 'red', x: 135, y: 165, status: 'pending' },
      { id: 6, color: 'yellow', x: 255, y: 255, status: 'pending' },
    ];
    BINS.forEach(b => (b.count = 0));
    sortedCaps = [];
    resetGantryPose();
    updateTelemetryUI();
    renderOnce();
  }

  function updateControlButtonsState() {
    if (uiEls.startBtn) {
      if (simState === 'SAFE_HALTED') {
        uiEls.startBtn.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8"/><path d="M3 3v5h5"/></svg><span>Re-Arm &amp; Home Gantry</span>`;
        uiEls.startBtn.className = 'w-full py-2.5 rounded-xl bg-amber-500 hover:bg-amber-400 text-slate-950 font-bold text-sm flex items-center justify-center gap-2 shadow-lg shadow-amber-500/25 transition-all';
      } else if (simState !== 'IDLE') {
        uiEls.startBtn.innerHTML = `<span class="w-2 h-2 rounded-full bg-current animate-pulse"></span><span>Sorting Active (${simState.replace(/_/g, ' ')})</span>`;
        uiEls.startBtn.className = defaultStartBtnClass + ' opacity-85 cursor-default';
      } else {
        uiEls.startBtn.innerHTML = defaultStartBtnHTML;
        uiEls.startBtn.className = defaultStartBtnClass;
        if (window.lucide) window.lucide.createIcons({ nodes: [uiEls.startBtn] });
      }
    }

    if (uiEls.commsBtn) {
      const canCut = simState !== 'IDLE' && simState !== 'SAFE_HALTED';
      uiEls.commsBtn.classList.toggle('opacity-55', !canCut);
    }
  }

  function getBoardCellFromPointer(e) {
    const rect = canvas.getBoundingClientRect();
    const mouseX = (e.clientX - rect.left) * (LOGICAL_W / rect.width);
    const mouseY = (e.clientY - rect.top) * (LOGICAL_H / rect.height);
    const boardX = (mouseX - PADDING_LEFT) / SCALE;
    const boardY = (mouseY - PADDING_TOP) / SCALE;

    if (boardX >= 0 && boardX < BOARD_SIZE_MM && boardY >= 0 && boardY < BOARD_SIZE_MM) {
      const col = Math.floor(boardX / CELL_SIZE_MM);
      const row = Math.floor(boardY / CELL_SIZE_MM);
      const snapX = col * CELL_SIZE_MM + CELL_SIZE_MM / 2;
      const snapY = row * CELL_SIZE_MM + CELL_SIZE_MM / 2;
      const existingIdx = caps.findIndex(c => Math.hypot(c.x - snapX, c.y - snapY) < 15);
      return { col, row, snapX, snapY, existingIdx };
    }
    return null;
  }

  canvas.addEventListener('mousemove', (e) => {
    if (simState !== 'IDLE') {
      if (hoverCell) {
        hoverCell = null;
      }
      return;
    }
    const cell = getBoardCellFromPointer(e);
    const changed =
      (cell === null) !== (hoverCell === null) ||
      (cell && hoverCell && (cell.col !== hoverCell.col || cell.row !== hoverCell.row));
    hoverCell = cell;
    canvas.style.cursor = cell ? 'pointer' : 'crosshair';
    if (changed) renderOnce();
  });

  canvas.addEventListener('mouseleave', () => {
    if (hoverCell) {
      hoverCell = null;
      if (simState === 'IDLE') renderOnce();
    }
  });

  canvas.addEventListener('click', (e) => {
    if (simState !== 'IDLE') {
      showNotification('Pause or wait for the current cycle to finish before editing caps.', 'info');
      return;
    }
    const cell = getBoardCellFromPointer(e);
    if (!cell) return;

    if (cell.existingIdx >= 0) {
      caps.splice(cell.existingIdx, 1);
    } else {
      caps.push({
        id: Date.now() + Math.random(),
        color: selectedColor,
        x: cell.snapX,
        y: cell.snapY,
        status: 'pending'
      });
    }
    hoverCell = getBoardCellFromPointer(e);
    updateTelemetryUI();
    renderOnce();
  });

  // Buttons
  uiEls.startBtn?.addEventListener('click', () => {
    if (simState === 'SAFE_HALTED') {
      resetGantryPose();
      updateTelemetryUI();
      renderOnce();
      showNotification('Gantry re-armed & homed to (0, 0). Ready for next sort cycle.', 'success');
      return;
    }
    startSorting();
  });

  document.getElementById('btn-reset-sim')?.addEventListener('click', () => {
    resetToDefaultCaps();
    showNotification('Simulator reset to 6 default bottle caps.', 'info');
  });

  document.getElementById('btn-preset-scatter')?.addEventListener('click', () => {
    caps = [];
    const colors = ['red', 'yellow', 'green', 'blue'];
    let i = 0;
    while (caps.length < 8) {
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
          status: 'pending'
        });
        i++;
      }
    }
    BINS.forEach(b => (b.count = 0));
    sortedCaps = [];
    resetGantryPose();
    updateTelemetryUI();
    renderOnce();
  });

  document.getElementById('btn-clear-sim')?.addEventListener('click', () => {
    caps = [];
    BINS.forEach(b => (b.count = 0));
    sortedCaps = [];
    resetGantryPose();
    updateTelemetryUI();
    renderOnce();
  });

  document.getElementById('btn-sim-speed')?.addEventListener('click', (e) => {
    const btn = e.currentTarget;
    if (simSpeedMultiplier === 1) {
      simSpeedMultiplier = 2;
      btn.textContent = 'Speed: 2x';
    } else if (simSpeedMultiplier === 2) {
      simSpeedMultiplier = 4;
      btn.textContent = 'Speed: 4x';
    } else {
      simSpeedMultiplier = 1;
      btn.textContent = 'Speed: 1x';
    }
  });

  uiEls.commsBtn?.addEventListener('click', () => {
    if (simState === 'IDLE') {
      showNotification('Start an autonomous sort first, then trigger a UART cut mid-cycle to test fail-safe halt.', 'info');
      return;
    }
    if (simState === 'SAFE_HALTED') return;
    commsLost = true;
    gantry.z = 'RAISED';
    gantry.zHeight = 0;
    simState = 'SAFE_HALTED';
    updateControlButtonsState();
    updateTelemetryUI();
    renderOnce();
    showNotification('UART Link Interrupted! Fail-safe halt latched: XY stopped, Z retracted (jaw preserved). Click "Re-Arm & Home" to recover.', 'error');
  });

  // Color picker buttons
  document.querySelectorAll('.cap-color-picker').forEach(el => {
    el.addEventListener('click', () => {
      document.querySelectorAll('.cap-color-picker').forEach(b => {
        b.classList.remove('ring-2', 'ring-white');
        b.setAttribute('aria-pressed', 'false');
      });
      el.classList.add('ring-2', 'ring-white');
      el.setAttribute('aria-pressed', 'true');
      selectedColor = el.dataset.color;
      if (hoverCell && simState === 'IDLE') renderOnce();
    });
  });

  function startSorting() {
    if (caps.length === 0) {
      showNotification('Click anywhere on the 10×10 grid to place bottle caps first!', 'info');
      return;
    }
    if (simState !== 'IDLE') return;

    hoverCell = null;
    simState = 'SCANNING';
    animTimer = 0;
    scanSweepY = 0;
    simElapsedSec = 0;
    commsLost = false;

    // Nearest-neighbor heuristic for shortest Cartesian travel
    const remaining = [...caps];
    planQueue = [];
    let curX = gantry.x;
    let curY = gantry.y;

    while (remaining.length > 0) {
      remaining.sort((a, b) => Math.hypot(a.x - curX, a.y - curY) - Math.hypot(b.x - curX, b.y - curY));
      const nextCap = remaining.shift();
      planQueue.push(nextCap);
      const bin = BINS.find(b => b.id === nextCap.color);
      curX = bin.x + bin.w / 2;
      curY = bin.y + bin.h / 2;
    }

    updateControlButtonsState();
    ensureLoopRunning();

    if (isV2) {
      showNotification(`Pi Zero 2 W OpenCV computed ${planQueue.length} cap targets in ~45 ms. Batch dispatched to Mega!`, 'success');
    } else {
      showNotification(`Teensy 4.1 computed ${planQueue.length} cap targets. Batch dispatched to Mega!`, 'success');
    }
  }

  function setGantryTarget(tx, ty) {
    gantry.startX = gantry.x;
    gantry.startY = gantry.y;
    gantry.targetX = tx;
    gantry.targetY = ty;
  }

  // Smooth trapezoidal ease-in / ease-out Cartesian travel
  function moveGantryTowards(tx, ty, dt) {
    const dx = tx - gantry.x;
    const dy = ty - gantry.y;
    const distRemaining = Math.hypot(dx, dy);
    const distFromStart = Math.hypot(gantry.x - gantry.startX, gantry.y - gantry.startY);
    const totalDist = Math.max(1, Math.hypot(tx - gantry.startX, ty - gantry.startY));

    // Smooth accel/decel ramp over first/last 28mm
    const rampDist = Math.min(28, totalDist * 0.35);
    let speedFactor = 1;
    if (distFromStart < rampDist) {
      speedFactor = Math.min(speedFactor, 0.32 + 0.68 * (distFromStart / Math.max(1, rampDist)));
    }
    if (distRemaining < rampDist) {
      speedFactor = Math.min(speedFactor, 0.25 + 0.75 * (distRemaining / Math.max(1, rampDist)));
    }

    const step = gantry.speed * speedFactor * dt;

    if (distRemaining <= step || distRemaining < 0.35) {
      gantry.x = tx;
      gantry.y = ty;
      return true;
    }
    gantry.x += (dx / distRemaining) * step;
    gantry.y += (dy / distRemaining) * step;
    return false;
  }

  function tick(timestamp) {
    const rawDt = Math.min((timestamp - lastFrameTime) / 1000, 0.08);
    lastFrameTime = timestamp;
    const dt = rawDt * simSpeedMultiplier;

    const prevState = simState;
    updateSimulation(dt);
    if (simState !== prevState) {
      updateControlButtonsState();
    }

    renderSimulation();
    syncCoordinatesUI();

    if (simState !== 'IDLE' && simState !== 'SAFE_HALTED') {
      rafId = requestAnimationFrame(tick);
    } else {
      rafId = null;
    }
  }

  function updateSimulation(dt) {
    if (simState === 'IDLE' || simState === 'SAFE_HALTED') return;
    simElapsedSec += dt;

    if (simState === 'SCANNING') {
      animTimer += dt;
      scanSweepY = Math.min(1, animTimer / 0.75);
      if (animTimer >= 0.85) {
        simState = 'NEXT_CAP';
      }
      return;
    }

    if (simState === 'NEXT_CAP') {
      if (planQueue.length === 0) {
        simState = 'PARKING';
        setGantryTarget(0, 0);
        return;
      }
      currentTarget = planQueue.shift();
      setGantryTarget(currentTarget.x, currentTarget.y);
      simState = 'MOVING_TO_CAP';
      return;
    }

    if (simState === 'MOVING_TO_CAP') {
      if (moveGantryTowards(gantry.targetX, gantry.targetY, dt)) {
        simState = 'LOWERING_Z';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'LOWERING_Z') {
      animTimer += dt * 3.6;
      const t = Math.min(1, animTimer);
      gantry.zHeight = t * t * (3 - 2 * t); // smoothstep
      if (t >= 1) {
        gantry.z = 'LOWERED';
        simState = 'GRIPPING';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'GRIPPING') {
      animTimer += dt * 4.2;
      const t = Math.min(1, animTimer);
      gantry.gripperWidth = 36 - 10 * (t * t * (3 - 2 * t));
      if (t >= 1) {
        gantry.gripper = 'CLOSED';
        gantry.heldCap = currentTarget;
        caps = caps.filter(c => c.id !== currentTarget.id);
        updateTelemetryUI();
        simState = 'LIFTING_Z';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'LIFTING_Z') {
      animTimer += dt * 3.6;
      const t = Math.min(1, animTimer);
      gantry.zHeight = 1 - (t * t * (3 - 2 * t));
      if (t >= 1) {
        gantry.zHeight = 0;
        gantry.z = 'RAISED';
        const bin = BINS.find(b => b.id === gantry.heldCap.color);
        setGantryTarget(bin.x + bin.w / 2, bin.y + bin.h / 2);
        simState = 'MOVING_TO_BIN';
      }
      return;
    }

    if (simState === 'MOVING_TO_BIN') {
      if (moveGantryTowards(gantry.targetX, gantry.targetY, dt)) {
        simState = 'LOWERING_BIN';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'LOWERING_BIN') {
      animTimer += dt * 3.6;
      const t = Math.min(1, animTimer / 0.65);
      gantry.zHeight = 0.65 * (t * t * (3 - 2 * t));
      if (t >= 1) {
        simState = 'RELEASING';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'RELEASING') {
      animTimer += dt * 4.5;
      const t = Math.min(1, animTimer);
      gantry.gripperWidth = 26 + 10 * (t * t * (3 - 2 * t));
      if (t >= 1) {
        gantry.gripper = 'OPEN';
        const bin = BINS.find(b => b.id === gantry.heldCap.color);
        bin.count++;
        sortedCaps.push({ ...gantry.heldCap });
        gantry.heldCap = null;
        updateTelemetryUI();
        simState = 'RETRACTING_BIN';
        animTimer = 0;
      }
      return;
    }

    if (simState === 'RETRACTING_BIN') {
      animTimer += dt * 3.8;
      const t = Math.min(1, animTimer / 0.65);
      gantry.zHeight = 0.65 * (1 - t * t * (3 - 2 * t));
      if (t >= 1) {
        gantry.zHeight = 0;
        gantry.z = 'RAISED';
        simState = 'NEXT_CAP';
      }
      return;
    }

    if (simState === 'PARKING') {
      if (moveGantryTowards(0, 0, dt)) {
        simState = 'IDLE';
        updateControlButtonsState();
        updateTelemetryUI();
        showNotification(`Batch complete! Sorted ${sortedCaps.length} caps in ${simElapsedSec.toFixed(1)}s simulated time.`, 'success');
      }
    }
  }

  function drawRoundedRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.roundRect(x, y, w, h, r);
  }

  function renderSimulation() {
    ctx.clearRect(0, 0, LOGICAL_W, LOGICAL_H);

    const boardPx = BOARD_SIZE_MM * SCALE;

    // 1. Workspace board backdrop
    drawRoundedRect(PADDING_LEFT, PADDING_TOP, boardPx, boardPx, 4);
    ctx.fillStyle = '#0f1626';
    ctx.fill();

    // Subtle 10x10 grid lines
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.12)';
    ctx.lineWidth = 1;
    for (let i = 1; i < GRID_DIVS; i++) {
      const xPos = PADDING_LEFT + i * (CELL_SIZE_MM * SCALE);
      ctx.beginPath();
      ctx.moveTo(xPos, PADDING_TOP);
      ctx.lineTo(xPos, PADDING_TOP + boardPx);
      ctx.stroke();

      const yPos = PADDING_TOP + i * (CELL_SIZE_MM * SCALE);
      ctx.beginPath();
      ctx.moveTo(PADDING_LEFT, yPos);
      ctx.lineTo(PADDING_LEFT + boardPx, yPos);
      ctx.stroke();
    }

    // Grid axis labels (A-J along top, 1-10 along left) positioned clear of home carriage
    ctx.fillStyle = '#64748b';
    ctx.font = '600 10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let i = 0; i < GRID_DIVS; i++) {
      const gx = PADDING_LEFT + (i + 0.5) * (CELL_SIZE_MM * SCALE);
      ctx.fillText(String.fromCharCode(65 + i), gx, PADDING_TOP - 22);
      const gy = PADDING_TOP + (i + 0.5) * (CELL_SIZE_MM * SCALE);
      ctx.fillText((i + 1).toString(), PADDING_LEFT - 24, gy);
    }

    // Board border & 4 corner homography fiducials
    ctx.strokeStyle = accentPrimary;
    ctx.lineWidth = 1.75;
    ctx.strokeRect(PADDING_LEFT, PADDING_TOP, boardPx, boardPx);

    const corners = [
      [PADDING_LEFT, PADDING_TOP],
      [PADDING_LEFT + boardPx, PADDING_TOP],
      [PADDING_LEFT, PADDING_TOP + boardPx],
      [PADDING_LEFT + boardPx, PADDING_TOP + boardPx]
    ];
    corners.forEach(([cx, cy]) => {
      ctx.fillStyle = accentPrimary;
      ctx.fillRect(cx - 4, cy - 4, 8, 8);
    });

    // Hover cell preview (when IDLE)
    if (hoverCell && simState === 'IDLE') {
      const hx = PADDING_LEFT + hoverCell.col * (CELL_SIZE_MM * SCALE);
      const hy = PADDING_TOP + hoverCell.row * (CELL_SIZE_MM * SCALE);
      const cellPx = CELL_SIZE_MM * SCALE;
      ctx.fillStyle = hoverCell.existingIdx >= 0 ? 'rgba(239, 68, 68, 0.16)' : 'rgba(255, 255, 255, 0.06)';
      ctx.fillRect(hx + 1, hy + 1, cellPx - 2, cellPx - 2);
      ctx.strokeStyle = hoverCell.existingIdx >= 0 ? 'rgba(248, 113, 113, 0.8)' : accentPrimary;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(hx + 1, hy + 1, cellPx - 2, cellPx - 2);

      if (hoverCell.existingIdx < 0) {
        const cx = PADDING_LEFT + hoverCell.snapX * SCALE;
        const cy = PADDING_TOP + hoverCell.snapY * SCALE;
        ctx.save();
        ctx.globalAlpha = 0.45;
        ctx.beginPath();
        ctx.arc(cx, cy, CAP_RADIUS_MM * SCALE, 0, Math.PI * 2);
        ctx.fillStyle = getCapHex(selectedColor);
        ctx.fill();
        ctx.restore();
      }
    }

    // Active planned path preview line (shows target trajectory!)
    if (simState === 'MOVING_TO_CAP' || simState === 'MOVING_TO_BIN' || simState === 'PARKING') {
      const gx = PADDING_LEFT + gantry.x * SCALE;
      const gy = PADDING_TOP + gantry.y * SCALE;
      const tx = PADDING_LEFT + gantry.targetX * SCALE;
      const ty = PADDING_TOP + gantry.targetY * SCALE;
      ctx.save();
      ctx.strokeStyle = simState === 'MOVING_TO_BIN' ? 'rgba(16, 185, 129, 0.65)' : accentPrimary;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(gx, gy);
      ctx.lineTo(tx, ty);
      ctx.stroke();
      ctx.restore();
    }

    // 2. Draw 4 Sorting Bins on right side (integrated header strip — never obscured by gantry beam!)
    BINS.forEach(bin => {
      const bx = PADDING_LEFT + bin.x * SCALE;
      const by = PADDING_TOP + bin.y * SCALE;
      const bw = bin.w * SCALE;
      const bh = bin.h * SCALE;
      const headerH = 20;

      // Bin body
      drawRoundedRect(bx, by, bw, bh, 8);
      ctx.fillStyle = bin.bg;
      ctx.fill();

      // Integrated colored top header bar inside bin
      ctx.save();
      drawRoundedRect(bx, by, bw, bh, 8);
      ctx.clip();
      ctx.fillStyle = bin.headerBg;
      ctx.fillRect(bx, by, bw, headerH);
      ctx.restore();

      // Bin outer stroke
      drawRoundedRect(bx, by, bw, bh, 8);
      ctx.strokeStyle = bin.color;
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Header text cleanly centered inside header strip
      ctx.fillStyle = '#f8fafc';
      ctx.font = '700 9.5px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(bin.name, bx + bw / 2, by + headerH / 2 + 0.5);

      // Mini sorted cap tokens inside bin body
      const dots = Math.min(bin.count, 8);
      for (let d = 0; d < dots; d++) {
        const col = d % 4;
        const row = Math.floor(d / 4);
        ctx.beginPath();
        ctx.arc(bx + 16 + col * 15, by + headerH + 14 + row * 14, 5, 0, Math.PI * 2);
        ctx.fillStyle = bin.color;
        ctx.fill();
        ctx.strokeStyle = 'rgba(255,255,255,0.5)';
        ctx.lineWidth = 1;
        ctx.stroke();
      }

      // Count badge at bottom of bin
      ctx.fillStyle = bin.count > 0 ? '#ffffff' : '#94a3b8';
      ctx.font = '600 10.5px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`Qty: ${bin.count}`, bx + bw / 2, by + bh - 12);
    });

    // 3. Draw Pending Caps on Board
    caps.forEach(cap => {
      const cx = PADDING_LEFT + cap.x * SCALE;
      const cy = PADDING_TOP + cap.y * SCALE;
      const cr = CAP_RADIUS_MM * SCALE;
      const isCurrent = currentTarget && currentTarget.id === cap.id;

      if (isCurrent) {
        ctx.beginPath();
        ctx.arc(cx, cy, cr + 5, 0, Math.PI * 2);
        ctx.strokeStyle = accentPrimary;
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }

      // Drop shadow
      ctx.beginPath();
      ctx.arc(cx + 1.5, cy + 2, cr, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
      ctx.fill();

      // Outer cap body
      ctx.beginPath();
      ctx.arc(cx, cy, cr, 0, Math.PI * 2);
      ctx.fillStyle = getCapHex(cap.color);
      ctx.fill();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.42)';
      ctx.lineWidth = 1.75;
      ctx.stroke();

      // Inner ribbed ridge
      ctx.beginPath();
      ctx.arc(cx, cy, cr * 0.68, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.28)';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Centroid crosshair
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx - 3.5, cy); ctx.lineTo(cx + 3.5, cy);
      ctx.moveTo(cx, cy - 3.5); ctx.lineTo(cx, cy + 3.5);
      ctx.stroke();
    });

    // 4. Camera scanning laser sweep overlay
    if (simState === 'SCANNING') {
      const sweepPxY = PADDING_TOP + scanSweepY * boardPx;
      const grad = ctx.createLinearGradient(0, sweepPxY - 34, 0, sweepPxY);
      grad.addColorStop(0, 'rgba(0, 210, 255, 0)');
      grad.addColorStop(1, 'rgba(0, 210, 255, 0.24)');
      ctx.fillStyle = grad;
      ctx.fillRect(PADDING_LEFT, Math.max(PADDING_TOP, sweepPxY - 34), boardPx, Math.min(34, sweepPxY - PADDING_TOP));

      ctx.strokeStyle = '#00d2ff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(PADDING_LEFT, sweepPxY);
      ctx.lineTo(PADDING_LEFT + boardPx, sweepPxY);
      ctx.stroke();

      // Clean centered pill banner inside board (never overflows into bins!)
      const label = isV2 ? 'PI ZERO 2 W OPENCV + FISHEYE SCAN...' : 'TEENSY 4.1 HSV + HOMOGRAPHY SCAN...';
      const pillW = 310;
      const pillH = 30;
      const pillX = PADDING_LEFT + (boardPx - pillW) / 2;
      const pillY = PADDING_TOP + (boardPx - pillH) / 2;
      drawRoundedRect(pillX, pillY, pillW, pillH, 999);
      ctx.fillStyle = 'rgba(10, 14, 23, 0.92)';
      ctx.fill();
      ctx.strokeStyle = '#00d2ff';
      ctx.lineWidth = 1.25;
      ctx.stroke();

      ctx.fillStyle = '#38bdf8';
      ctx.font = '700 11px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, pillX + pillW / 2, pillY + pillH / 2);
    }

    // 5. Gantry Rails & Moving Carriage
    const gantryPxX = PADDING_LEFT + gantry.x * SCALE;
    const gantryPxY = PADDING_TOP + gantry.y * SCALE;

    // Y-Axis Gantry Bridge Beam (subtle frosted rail spanning board + bins)
    ctx.fillStyle = 'rgba(30, 41, 59, 0.42)';
    drawRoundedRect(PADDING_LEFT - 10, gantryPxY - 5, boardPx + 132, 10, 3);
    ctx.fill();
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Z-Linkage Lowering Halo
    if (gantry.zHeight > 0.04) {
      ctx.fillStyle = `rgba(0, 210, 255, ${gantry.zHeight * 0.42})`;
      ctx.beginPath();
      ctx.arc(gantryPxX, gantryPxY, 22 * gantry.zHeight + 8, 0, Math.PI * 2);
      ctx.fill();
    }

    // X-Axis Carriage Block
    drawRoundedRect(gantryPxX - 13, gantryPxY - 13, 26, 26, 6);
    ctx.fillStyle = simState === 'SAFE_HALTED' ? '#ef4444' : accentPrimary;
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // 2-Finger Parallel Gripper Jaws
    const jawHalfGap = (gantry.gripperWidth * SCALE) / 2;
    ctx.fillStyle = '#f8fafc';
    drawRoundedRect(gantryPxX - jawHalfGap - 4.5, gantryPxY - 12, 4.5, 24, 2);
    ctx.fill();
    drawRoundedRect(gantryPxX + jawHalfGap, gantryPxY - 12, 4.5, 24, 2);
    ctx.fill();

    // Held Cap
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

    // 6. Sleek Bottom HUD Status Strip (fills bottom area cleanly with live context)
    const hudY = 434;
    const hudH = 48;
    drawRoundedRect(18, hudY, LOGICAL_W - 36, hudH, 10);
    ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
    ctx.fill();
    ctx.strokeStyle = simState === 'SAFE_HALTED' ? 'rgba(239, 68, 68, 0.5)' : 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Left HUD status text
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.font = '600 11px "JetBrains Mono", monospace';
    let hudLeft = '';
    if (simState === 'SAFE_HALTED') {
      ctx.fillStyle = '#f87171';
      hudLeft = 'FAULT LATCHED • Z RETRACTED • CLICK "RE-ARM & HOME"';
    } else if (simState !== 'IDLE') {
      ctx.fillStyle = '#34d399';
      hudLeft = `ACTIVE: ${simState.replace(/_/g, ' ')}  •  ELAPSED ${simElapsedSec.toFixed(1)}s`;
    } else if (hoverCell) {
      const colChar = String.fromCharCode(65 + hoverCell.col);
      const action = hoverCell.existingIdx >= 0 ? 'Click to remove' : `Click to place ${selectedColor.toUpperCase()}`;
      ctx.fillStyle = '#e2e8f0';
      hudLeft = `CELL ${colChar}${hoverCell.row + 1} (${hoverCell.snapX}, ${hoverCell.snapY} mm) • ${action}`;
    } else {
      ctx.fillStyle = '#94a3b8';
      hudLeft = 'READY • Hover grid to place/remove caps or Start Sort';
    }
    ctx.fillText(hudLeft, 32, hudY + hudH / 2);

    // Right HUD home/origin badge
    ctx.textAlign = 'right';
    ctx.fillStyle = accentPrimary;
    ctx.font = '700 10.5px "JetBrains Mono", monospace';
    ctx.fillText(`(${Math.round(gantry.x)}, ${Math.round(gantry.y)}) mm`, LOGICAL_W - 32, hudY + hudH / 2);
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

  function syncCoordinatesUI() {
    const rx = `${Math.round(gantry.x)} mm`;
    const ry = `${Math.round(gantry.y)} mm`;
    const rz = gantry.z;
    const rg = gantry.gripper;
    const rs = simState;

    if (uiEls.x && lastTelemetry.x !== rx) {
      uiEls.x.textContent = rx;
      lastTelemetry.x = rx;
    }
    if (uiEls.y && lastTelemetry.y !== ry) {
      uiEls.y.textContent = ry;
      lastTelemetry.y = ry;
    }
    if (uiEls.z && lastTelemetry.z !== rz) {
      uiEls.z.textContent = rz;
      lastTelemetry.z = rz;
    }
    if (uiEls.grip && lastTelemetry.grip !== rg) {
      uiEls.grip.textContent = rg;
      lastTelemetry.grip = rg;
    }
    if (uiEls.state && lastTelemetry.state !== rs) {
      uiEls.state.textContent = rs.replace(/_/g, ' ');
      uiEls.state.className =
        rs === 'SAFE_HALTED'
          ? 'text-red-400 font-bold'
          : rs === 'IDLE'
          ? 'text-slate-400 font-mono'
          : 'text-emerald-400 font-bold';
      lastTelemetry.state = rs;
    }
  }

  function updateTelemetryUI() {
    if (uiEls.total) uiEls.total.textContent = String(caps.length + sortedCaps.length);
    if (uiEls.sorted) uiEls.sorted.textContent = String(sortedCaps.length);
    if (uiEls.pending) uiEls.pending.textContent = String(caps.length);
  }

  // Initial draw
  resetToDefaultCaps();
}

/* ==========================================================================
   2. Master Infographic Poster Modal (Lazy Load, Zero-Lag Pan, Pinch & Double-Click)
   ========================================================================== */
function initPosterModal() {
  const modal = document.getElementById('poster-modal');
  const openBtns = document.querySelectorAll('.btn-open-poster');
  const closeBtn = document.getElementById('btn-close-poster');
  const posterImg = document.getElementById('modal-poster-img');
  const zoomIn = document.getElementById('btn-zoom-in');
  const zoomOut = document.getElementById('btn-zoom-out');
  const zoomReset = document.getElementById('btn-zoom-reset');
  const btnV1 = document.getElementById('btn-poster-v1');
  const btnV2 = document.getElementById('btn-poster-v2');
  const viewport = modal?.querySelector('.poster-viewport');

  if (!modal || !posterImg || !viewport) return;

  posterImg.setAttribute('draggable', 'false');
  modal.setAttribute('role', 'dialog');
  modal.setAttribute('aria-modal', 'true');
  modal.setAttribute('aria-label', 'High-Resolution System Architecture Poster Viewer');

  // Inject smooth loading spinner into viewport
  const loader = document.createElement('div');
  loader.className = 'poster-loader';
  loader.innerHTML = `<div class="px-4 py-2 rounded-full bg-slate-900/90 border border-white/15 text-xs font-mono text-slate-200 flex items-center gap-2 shadow-xl"><span class="w-3.5 h-3.5 rounded-full border-2 border-cyan-400 border-t-transparent animate-spin"></span><span>Loading 4K UHD Poster…</span></div>`;
  viewport.appendChild(loader);

  // Inject subtle hint pill at bottom of viewport
  const hint = document.createElement('div');
  hint.className = 'absolute bottom-3 left-1/2 -translate-x-1/2 px-3 py-1 rounded-full bg-slate-900/85 border border-white/10 text-[11px] font-mono text-slate-400 pointer-events-none hidden sm:flex items-center gap-3 backdrop-blur-md';
  hint.innerHTML = `<span>Scroll or double-click to zoom</span><span>•</span><span>Drag to pan</span><span>•</span><span>Esc to close</span>`;
  viewport.appendChild(hint);

  // Inject live zoom readout & download link into toolbar if not present
  const zoomBadge = document.getElementById('poster-zoom-level');
  const downloadLink = document.getElementById('btn-download-poster');

  let scale = 1;
  let isPanning = false;
  let didDrag = false;
  let startX = 0, startY = 0;
  let downX = 0, downY = 0;
  let translateX = 0, translateY = 0;
  let lastTrigger = null;
  let pinchStartDist = 0;
  let pinchStartScale = 1;

  function updateTransform() {
    if (scale <= 1.01) {
      translateX = 0;
      translateY = 0;
    }
    posterImg.style.transform = `translate3d(${translateX}px, ${translateY}px, 0) scale(${scale})`;
    viewport.classList.toggle('is-zoomed', scale > 1.02);
    if (zoomBadge) {
      zoomBadge.textContent = `${Math.round(scale * 100)}%`;
    }
  }

  function loadPosterSrc(src) {
    if (!src) return;
    const currentAttr = posterImg.getAttribute('src');
    if (currentAttr !== src) {
      posterImg.classList.add('is-loading');
      loader.classList.add('visible');
      posterImg.onload = () => {
        posterImg.classList.remove('is-loading');
        loader.classList.remove('visible');
      };
      posterImg.src = src;
    }
    if (downloadLink) {
      downloadLink.href = src;
    }
    updateToggleButtons(src);
  }

  function updateToggleButtons(currentSrc) {
    const isV2Poster = (currentSrc || '').includes('_v2');
    if (btnV1 && btnV2) {
      if (isV2Poster) {
        btnV2.className = 'px-2.5 py-1 text-xs rounded-md bg-cyan-500 text-slate-950 font-bold transition-all shadow-sm';
        btnV1.className = 'px-2.5 py-1 text-xs rounded-md bg-slate-800 text-slate-300 hover:text-white border border-slate-700 transition-all';
      } else {
        btnV1.className = 'px-2.5 py-1 text-xs rounded-md bg-orange-500 text-white font-bold transition-all shadow-sm';
        btnV2.className = 'px-2.5 py-1 text-xs rounded-md bg-slate-800 text-slate-300 hover:text-white border border-slate-700 transition-all';
      }
    }
  }

  function resetView() {
    scale = 1;
    translateX = 0;
    translateY = 0;
    updateTransform();
  }

  openBtns.forEach(btn => {
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      lastTrigger = btn;
      const desiredSrc =
        btn.dataset.poster ||
        posterImg.dataset.src ||
        posterImg.getAttribute('src') ||
        (isV2Theme() ? 'assets/system_poster_v2.jpg' : 'assets/system_poster.jpg');
      loadPosterSrc(desiredSrc);
      resetView();
      modal.classList.add('open');
      document.body.style.overflow = 'hidden';
      closeBtn?.focus({ preventScroll: true });
    });
  });

  btnV1?.addEventListener('click', () => {
    resetView();
    loadPosterSrc('assets/system_poster.jpg');
  });

  btnV2?.addEventListener('click', () => {
    resetView();
    loadPosterSrc('assets/system_poster_v2.jpg');
  });

  function closeModal() {
    modal.classList.remove('open');
    document.body.style.overflow = '';
    if (lastTrigger && typeof lastTrigger.focus === 'function') {
      lastTrigger.focus({ preventScroll: true });
    }
  }

  closeBtn?.addEventListener('click', closeModal);

  // Close only on genuine background click (NEVER right after dragging!)
  modal.addEventListener('click', (e) => {
    if (didDrag) {
      didDrag = false;
      return;
    }
    if (e.target === modal || e.target === viewport) {
      closeModal();
    }
  });

  document.addEventListener('keydown', (e) => {
    if (!modal.classList.contains('open')) return;
    if (e.key === 'Escape') {
      closeModal();
    } else if (e.key === '+' || e.key === '=') {
      scale = Math.min(3.5, scale + 0.35);
      updateTransform();
    } else if (e.key === '-') {
      scale = Math.max(0.7, scale - 0.35);
      updateTransform();
    } else if (e.key === '0') {
      resetView();
    }
  });

  zoomIn?.addEventListener('click', () => {
    scale = Math.min(3.5, scale + 0.4);
    updateTransform();
  });

  zoomOut?.addEventListener('click', () => {
    scale = Math.max(0.7, scale - 0.4);
    updateTransform();
  });

  zoomReset?.addEventListener('click', resetView);

  // Double-click to toggle crisp 2.2x zoom
  posterImg.addEventListener('dblclick', (e) => {
    e.preventDefault();
    if (scale > 1.15) {
      resetView();
    } else {
      scale = 2.2;
      translateX = 0;
      translateY = 0;
      updateTransform();
    }
  });

  // Zero-lag Mouse Panning
  viewport.addEventListener('mousedown', (e) => {
    if (e.button !== 0 || scale <= 1.02) return;
    e.preventDefault();
    isPanning = true;
    didDrag = false;
    downX = e.clientX;
    downY = e.clientY;
    startX = e.clientX - translateX;
    startY = e.clientY - translateY;
    posterImg.classList.add('is-dragging');
    viewport.classList.add('is-dragging');
  });

  window.addEventListener('mousemove', (e) => {
    if (!isPanning) return;
    if (Math.hypot(e.clientX - downX, e.clientY - downY) > 4) {
      didDrag = true;
    }
    translateX = e.clientX - startX;
    translateY = e.clientY - startY;
    updateTransform();
  });

  window.addEventListener('mouseup', () => {
    if (!isPanning) return;
    isPanning = false;
    posterImg.classList.remove('is-dragging');
    viewport.classList.remove('is-dragging');
    if (didDrag) {
      setTimeout(() => { didDrag = false; }, 60);
    }
  });

  // Touch Pan & Pinch Zoom
  viewport.addEventListener('touchstart', (e) => {
    if (e.touches.length === 2) {
      pinchStartDist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
      pinchStartScale = scale;
      posterImg.classList.add('is-dragging');
    } else if (e.touches.length === 1 && scale > 1.02) {
      isPanning = true;
      startX = e.touches[0].clientX - translateX;
      startY = e.touches[0].clientY - translateY;
      posterImg.classList.add('is-dragging');
    }
  }, { passive: true });

  viewport.addEventListener('touchmove', (e) => {
    if (e.touches.length === 2 && pinchStartDist > 0) {
      e.preventDefault();
      const dist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
      scale = Math.min(3.5, Math.max(0.7, pinchStartScale * (dist / pinchStartDist)));
      updateTransform();
    } else if (e.touches.length === 1 && isPanning) {
      e.preventDefault();
      translateX = e.touches[0].clientX - startX;
      translateY = e.touches[0].clientY - startY;
      updateTransform();
    }
  }, { passive: false });

  viewport.addEventListener('touchend', () => {
    isPanning = false;
    pinchStartDist = 0;
    posterImg.classList.remove('is-dragging');
  });

  // Smooth Wheel Zoom
  viewport.addEventListener('wheel', (e) => {
    e.preventDefault();
    const delta = e.deltaY * -0.0018;
    scale = Math.min(Math.max(0.7, scale + delta), 3.5);
    updateTransform();
  }, { passive: false });
}

/* ==========================================================================
   3. Vision Pipeline Step-by-Step Demonstrator (Smooth Transitions + V2 Theme)
   ========================================================================== */
function initVisionPipeline() {
  const stepBtns = document.querySelectorAll('.vision-step-btn');
  const stepPanels = document.querySelectorAll('.vision-step-panel');
  if (stepBtns.length === 0) return;

  const activeClasses = isV2Theme()
    ? ['bg-cyan-500/20', 'text-cyan-300', 'border-cyan-500/45']
    : ['bg-orange-500/20', 'text-orange-400', 'border-orange-500/40'];
  const allActiveCleanup = [
    'bg-orange-500/20', 'text-orange-400', 'border-orange-500/40',
    'bg-cyan-500/20', 'text-cyan-300', 'border-cyan-500/45'
  ];
  const inactiveClasses = ['bg-slate-800/40', 'text-slate-400', 'border-slate-700/40'];

  // Ensure initial active button reflects V1/V2 theme
  const firstBtn = stepBtns[0];
  if (firstBtn && isV2Theme()) {
    firstBtn.classList.remove(...allActiveCleanup, ...inactiveClasses);
    firstBtn.classList.add(...activeClasses);
  }

  stepBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      const targetStep = btn.dataset.step;

      stepBtns.forEach(b => {
        b.classList.remove(...allActiveCleanup);
        b.classList.add(...inactiveClasses);
      });
      btn.classList.remove(...inactiveClasses);
      btn.classList.add(...activeClasses);

      stepPanels.forEach(panel => {
        if (panel.id === `vision-panel-${targetStep}`) {
          panel.classList.remove('hidden', 'panel-animate-in');
          void panel.offsetWidth;
          panel.classList.add('panel-animate-in');
        } else {
          panel.classList.add('hidden');
        }
      });
    });
  });
}

/* ==========================================================================
   4 & 5. Enhanced Searchable / Filterable Tables (Empty State, Clear & Counter)
   ========================================================================== */
function enhanceFilterableTable({ inputId, filterSelector, tbodyId, dataAttr, itemLabel }) {
  const searchInput = document.getElementById(inputId);
  const filterBtns = Array.from(document.querySelectorAll(filterSelector));
  const tbody = document.getElementById(tbodyId);
  if (!tbody) return;

  const rows = Array.from(tbody.querySelectorAll('tr'));
  const totalCount = rows.length;
  const colSpan = tbody.closest('table')?.querySelectorAll('thead th').length || 6;

  const activeClasses = isV2Theme()
    ? ['active', 'bg-cyan-500/20', 'text-cyan-300', 'border-cyan-500/45']
    : ['active', 'bg-orange-500/20', 'text-orange-400', 'border-orange-500/40'];
  const allActiveCleanup = [
    'active',
    'bg-orange-500/20', 'text-orange-400', 'border-orange-500/40',
    'bg-cyan-500/20', 'text-cyan-300', 'border-cyan-500/45'
  ];
  const inactiveClasses = ['bg-slate-800', 'text-slate-300', 'border-slate-700'];

  // Apply V2 theme to initially active filter button
  if (isV2Theme()) {
    const initialActive = filterBtns.find(b => b.classList.contains('active'));
    if (initialActive) {
      initialActive.classList.remove(...allActiveCleanup, ...inactiveClasses);
      initialActive.classList.add(...activeClasses);
    }
  }

  // Inject clear button inside search input wrapper
  let clearBtn = null;
  if (searchInput && searchInput.parentElement) {
    searchInput.classList.add('pr-7');
    clearBtn = document.createElement('button');
    clearBtn.type = 'button';
    clearBtn.setAttribute('aria-label', 'Clear search');
    clearBtn.className = 'hidden absolute right-2 top-1/2 -translate-y-1/2 text-slate-400 hover:text-white p-0.5 rounded transition-colors';
    clearBtn.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M18 6 6 18"/><path d="m6 6 12 12"/></svg>`;
    clearBtn.addEventListener('click', () => {
      searchInput.value = '';
      filterTable();
      searchInput.focus();
    });
    searchInput.parentElement.appendChild(clearBtn);
  }

  // Inject subtle live count status strip below table container header or empty-state row
  const emptyRow = document.createElement('tr');
  emptyRow.className = 'hidden';
  emptyRow.innerHTML = `
    <td colspan="${colSpan}" class="py-10 text-center">
      <div class="flex flex-col items-center gap-2 text-slate-400">
        <span class="text-sm font-medium text-slate-300">No matching ${itemLabel} found</span>
        <span class="text-xs text-slate-500">Try a different keyword or reset the active category filter.</span>
        <button type="button" class="btn-reset-table-filter mt-1 px-3 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-xs font-mono text-slate-200 border border-white/10 transition-colors">Reset Filters</button>
      </div>
    </td>
  `;
  tbody.appendChild(emptyRow);

  emptyRow.querySelector('.btn-reset-table-filter')?.addEventListener('click', () => {
    if (searchInput) searchInput.value = '';
    const allBtn = filterBtns.find(b => b.dataset.filter === 'all') || filterBtns[0];
    allBtn?.click();
  });

  function filterTable() {
    const query = (searchInput?.value || '').trim().toLowerCase();
    if (clearBtn) clearBtn.classList.toggle('hidden', query.length === 0);

    const activeFilter = filterBtns.find(b => b.classList.contains('active'))?.dataset.filter || 'all';
    let visibleCount = 0;

    rows.forEach(row => {
      const text = row.textContent.toLowerCase();
      const attrVal = row.dataset[dataAttr] || '';
      const matchesSearch = !query || text.includes(query);
      const matchesFilter = activeFilter === 'all' || attrVal === activeFilter;

      if (matchesSearch && matchesFilter) {
        row.style.display = '';
        visibleCount++;
      } else {
        row.style.display = 'none';
      }
    });

    emptyRow.classList.toggle('hidden', visibleCount > 0);
  }

  searchInput?.addEventListener('input', filterTable);

  filterBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      filterBtns.forEach(b => {
        b.classList.remove(...allActiveCleanup);
        b.classList.add(...inactiveClasses);
      });
      btn.classList.remove(...inactiveClasses);
      btn.classList.add(...activeClasses);
      filterTable();
    });
  });

  // Return count getter if needed
  return { totalCount };
}

function initPinoutTable() {
  enhanceFilterableTable({
    inputId: 'pinout-search',
    filterSelector: '.pinout-filter-btn',
    tbodyId: 'pinout-table-body',
    dataAttr: 'controller',
    itemLabel: 'pin connections'
  });
}

function initBOMTable() {
  enhanceFilterableTable({
    inputId: 'bom-search',
    filterSelector: '.bom-filter-btn',
    tbodyId: 'bom-table-body',
    dataAttr: 'status',
    itemLabel: 'BOM components'
  });
}

/* ==========================================================================
   6. Firmware Code Vault (IDE Syntax Highlighting, Tabs & Resilient Copy)
   ========================================================================== */
function escapeHTML(str) {
  return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

function highlightCodeText(raw, lang) {
  const isPy = lang === 'python';
  const tokenRegex = isPy
    ? /(#.*$|"""[\s\S]*?"""|'''[\s\S]*?'''|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\b(?:import|from|as|def|class|return|if|elif|else|for|while|in|not|and|or|try|except|finally|with|pass|break|continue|True|False|None|raise|yield|lambda|global)\b|\b(?:int|float|str|bool|bytes|bytearray|list|dict|tuple|set|np|cv2|serial|time|os|sys|math|struct|argparse)\b|\b(?:0x[0-9a-fA-F]+|\d+(?:\.\d+)?)\b)/gm
    : /(\/\/.*$|\/\*[\s\S]*?\*\/|^\s*#\s*(?:include|define|ifdef|ifndef|endif|if|else|pragma)\b[^\n]*|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\b(?:const|static|volatile|struct|enum|union|typedef|return|if|else|for|while|do|switch|case|break|continue|default|sizeof|inline|extern|namespace|class|public|private|true|false|nullptr|NULL)\b|\b(?:void|bool|char|int|long|short|float|double|uint8_t|uint16_t|uint32_t|int8_t|int16_t|int32_t|size_t|unsigned|signed|DMAMEM|PROGMEM|String|HardwareSerial)\b|\b(?:0x[0-9a-fA-F]+|\d+(?:\.\d+)?[fFuUlL]*)\b)/gm;

  let out = '';
  let lastIdx = 0;
  let match;

  while ((match = tokenRegex.exec(raw)) !== null) {
    if (match.index > lastIdx) {
      out += escapeHTML(raw.slice(lastIdx, match.index));
    }
    const tok = match[0];
    let cls = '';
    if ((isPy && tok.startsWith('#')) || (!isPy && (tok.startsWith('//') || tok.startsWith('/*')))) {
      cls = 'tok-comment';
    } else if (!isPy && /^\s*#/.test(tok)) {
      cls = 'tok-preproc';
    } else if (tok.startsWith('"') || tok.startsWith("'")) {
      cls = 'tok-str';
    } else if (/^(?:0x|\d)/.test(tok)) {
      cls = 'tok-num';
    } else if (
      isPy
        ? /^(?:int|float|str|bool|bytes|bytearray|list|dict|tuple|set|np|cv2|serial|time|os|sys|math|struct|argparse)$/.test(tok)
        : /^(?:void|bool|char|int|long|short|float|double|uint8_t|uint16_t|uint32_t|int8_t|int16_t|int32_t|size_t|unsigned|signed|DMAMEM|PROGMEM|String|HardwareSerial)$/.test(tok)
    ) {
      cls = 'tok-type';
    } else {
      cls = 'tok-kw';
    }
    out += `<span class="${cls}">${escapeHTML(tok)}</span>`;
    lastIdx = tokenRegex.lastIndex;
  }

  if (lastIdx < raw.length) {
    out += escapeHTML(raw.slice(lastIdx));
  }
  return out;
}

function initCodeVault() {
  const tabs = document.querySelectorAll('.code-tab-btn');
  const blocks = document.querySelectorAll('.code-block');
  const copyBtns = document.querySelectorAll('.btn-copy-code');
  if (tabs.length === 0) return;

  const activeClasses = isV2Theme()
    ? ['active', 'text-cyan-300', 'border-cyan-400', 'bg-cyan-500/10']
    : ['active', 'text-orange-400', 'border-orange-500', 'bg-orange-500/10'];
  const allActiveCleanup = [
    'active',
    'text-orange-400', 'border-orange-500', 'bg-orange-500/10',
    'text-cyan-300', 'border-cyan-400', 'bg-cyan-500/10'
  ];

  // Style initial active tab for V2
  if (isV2Theme()) {
    const initialTab = document.querySelector('.code-tab-btn.active');
    if (initialTab) {
      initialTab.classList.remove(...allActiveCleanup, 'text-slate-400', 'border-transparent');
      initialTab.classList.add(...activeClasses);
    }
  }

  // Lazy-highlight each block on first view so page load stays instantaneous!
  function ensureHighlighted(block) {
    if (!block || block.dataset.highlighted === '1') return;
    const codeEl = block.querySelector('code');
    if (!codeEl) return;
    const raw = codeEl.textContent || '';
    codeEl.dataset.rawCode = raw;
    const isPy = block.id.includes('pizero') || block.id.includes('calibrate');
    codeEl.innerHTML = highlightCodeText(raw, isPy ? 'python' : 'cpp');
    block.dataset.highlighted = '1';
  }

  // Highlight initial visible block cleanly on idle
  const initialBlock = document.querySelector('.code-block:not(.hidden)');
  if (initialBlock) {
    const idle = window.requestIdleCallback || ((fn) => setTimeout(fn, 60));
    idle(() => ensureHighlighted(initialBlock));
  }

  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      const targetId = tab.dataset.code;

      tabs.forEach(t => {
        t.classList.remove(...allActiveCleanup);
        t.classList.add('text-slate-400', 'border-transparent');
      });
      tab.classList.remove('text-slate-400', 'border-transparent');
      tab.classList.add(...activeClasses);

      blocks.forEach(block => {
        if (block.id === `code-${targetId}`) {
          ensureHighlighted(block);
          block.classList.remove('hidden', 'panel-animate-in');
          void block.offsetWidth;
          block.classList.add('panel-animate-in');
        } else {
          block.classList.add('hidden');
        }
      });

      copyBtns.forEach(btn => {
        btn.dataset.target = `code-${targetId}`;
      });
    });
  });

  // Resilient Copy Code Handler
  copyBtns.forEach(btn => {
    let copyTimeout = null;
    const defaultHTML = btn.innerHTML;

    btn.addEventListener('click', async () => {
      const targetBlockId = btn.dataset.target || '';
      const codeEl =
        (targetBlockId && document.querySelector(`#${targetBlockId} code`)) ||
        document.querySelector('.code-block:not(.hidden) code');
      if (!codeEl) return;

      const textToCopy = codeEl.dataset.rawCode || codeEl.textContent || '';
      try {
        if (navigator.clipboard && navigator.clipboard.writeText) {
          await navigator.clipboard.writeText(textToCopy);
        } else {
          const ta = document.createElement('textarea');
          ta.value = textToCopy;
          ta.style.position = 'fixed';
          ta.style.opacity = '0';
          document.body.appendChild(ta);
          ta.select();
          document.execCommand('copy');
          ta.remove();
        }

        const labelName = targetBlockId ? targetBlockId.replace('code-', '') : 'active';
        showNotification(`Copied ${labelName} firmware to clipboard!`, 'success');
        if (copyTimeout) clearTimeout(copyTimeout);
        btn.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" class="text-emerald-400"><path d="M20 6 9 17l-5-5"/></svg><span class="text-emerald-400 font-bold">Copied</span>`;
        copyTimeout = setTimeout(() => {
          btn.innerHTML = defaultHTML;
        }, 2000);
      } catch {
        showNotification('Clipboard permission blocked — select code manually to copy.', 'warning');
      }
    });
  });
}

/* ==========================================================================
   7. Stacked UI Notification Toasts & Smooth Scrolling
   ========================================================================== */
function getToastContainer() {
  let container = document.getElementById('toast-container');
  if (!container) {
    container = document.createElement('div');
    container.id = 'toast-container';
    container.setAttribute('aria-live', 'polite');
    document.body.appendChild(container);
  }
  return container;
}

function showNotification(msg, type = 'info') {
  const container = getToastContainer();

  // Keep stack tidy (max 3 simultaneous toasts)
  while (container.children.length >= 3) {
    container.firstElementChild?.remove();
  }

  const toast = document.createElement('div');
  const styles = {
    success: { border: 'border-emerald-500/70', text: 'text-emerald-200', dot: 'bg-emerald-400' },
    warning: { border: 'border-amber-500/70', text: 'text-amber-200', dot: 'bg-amber-400' },
    error: { border: 'border-red-500/70', text: 'text-red-200', dot: 'bg-red-400' },
    info: { border: 'border-cyan-500/70', text: 'text-cyan-200', dot: 'bg-cyan-400' }
  }[type] || { border: 'border-cyan-500/70', text: 'text-cyan-200', dot: 'bg-cyan-400' };

  toast.className = `toast-item flex items-center gap-3 px-4 py-3 rounded-xl bg-slate-900/95 border ${styles.border} shadow-2xl backdrop-blur-md translate-y-3 opacity-0`;
  toast.innerHTML = `
    <span class="w-2.5 h-2.5 rounded-full ${styles.dot} shrink-0"></span>
    <span class="text-xs sm:text-sm font-medium ${styles.text} leading-snug flex-1">${escapeHTML(String(msg))}</span>
    <button type="button" aria-label="Dismiss notification" class="text-slate-400 hover:text-white p-0.5 shrink-0 transition-colors">
      <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M18 6 6 18"/><path d="m6 6 12 12"/></svg>
    </button>
  `;

  function dismiss() {
    toast.classList.add('translate-y-2', 'opacity-0');
    setTimeout(() => toast.remove(), 260);
  }

  toast.querySelector('button')?.addEventListener('click', dismiss);
  container.appendChild(toast);

  requestAnimationFrame(() => {
    toast.classList.remove('translate-y-3', 'opacity-0');
  });

  setTimeout(dismiss, 4200);
}

// Programmatic scroll lock so scroll-spy doesn't flicker through intermediate pills on click
let navLockUntil = 0;
let engLockUntil = 0;

function initSmoothScroll() {
  document.querySelectorAll('a[href^="#"]:not(.eng-pill-link)').forEach(anchor => {
    anchor.addEventListener('click', function (e) {
      const href = this.getAttribute('href');
      if (!href || href === '#') return;

      const engHashes = ['#engineering', '#vision', '#electrical', '#safety', '#code', '#considerations'];
      if (engHashes.includes(href) && typeof window.openEngineeringModule === 'function') {
        e.preventDefault();
        const mod = href.substring(1);
        navLockUntil = performance.now() + 650;
        if (typeof window.setActiveTopNavSection === 'function') {
          window.setActiveTopNavSection('engineering');
        }
        window.openEngineeringModule(mod, true);
        return;
      }

      const target = document.querySelector(href);
      if (target) {
        e.preventDefault();
        navLockUntil = performance.now() + 650;
        const secId = href.substring(1);
        if (typeof window.setActiveTopNavSection === 'function') {
          window.setActiveTopNavSection(secId);
        }
        target.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    });
  });
}

/* ==========================================================================
   8. On-Demand Engineering Deep-Dive Drawer
   ========================================================================== */
function initEngineeringDrawer() {
  const drawer = document.getElementById('engineering-drawer');
  const toggleBtn = document.getElementById('btn-toggle-engineering');
  const toggleLabel = document.getElementById('btn-toggle-engineering-label');
  const toggleIcon = document.getElementById('btn-toggle-engineering-icon');
  const closeBtn = document.getElementById('btn-close-engineering');
  const quickCards = document.querySelectorAll('.eng-quick-card');
  const modules = document.querySelectorAll('.eng-module-panel');
  const engSection = document.getElementById('engineering');
  const backToTopBtn = document.getElementById('btn-eng-back-top');
  const collapseBottomBtn = document.getElementById('btn-eng-collapse-bottom');

  if (!drawer || !engSection) return;

  let isOpen = false;
  toggleBtn?.setAttribute('aria-expanded', 'false');

  function setDrawerOpen(openState, targetModuleId = null, shouldScroll = false) {
    const wasOpen = isOpen;
    isOpen = openState;
    toggleBtn?.setAttribute('aria-expanded', String(isOpen));
    toggleIcon?.classList.toggle('rotated', isOpen);

    if (isOpen) {
      drawer.classList.remove('hidden');
      if (!wasOpen) {
        drawer.classList.remove('is-opening');
        void drawer.offsetWidth;
        drawer.classList.add('is-opening');
        const clearOpening = () => drawer.classList.remove('is-opening');
        drawer.addEventListener('animationend', clearOpening, { once: true });
        setTimeout(clearOpening, 450);
      }
      modules.forEach(mod => mod.classList.remove('hidden'));
      if (toggleLabel) {
        toggleLabel.textContent = 'Hide Engineering Deep-Dive';
      }
      if (typeof window.refreshEngineeringScrollSpy === 'function') {
        window.refreshEngineeringScrollSpy(true);
      }
      if (shouldScroll) {
        requestAnimationFrame(() => {
          const scrollTarget =
            targetModuleId &&
            targetModuleId !== 'engineering' &&
            targetModuleId !== 'all' &&
            document.getElementById(targetModuleId)
              ? document.getElementById(targetModuleId)
              : engSection;

          if (targetModuleId && targetModuleId !== 'engineering' && typeof window.setActiveEngSection === 'function') {
            engLockUntil = performance.now() + 650;
            window.setActiveEngSection(targetModuleId);
          }
          scrollTarget.scrollIntoView({ behavior: 'smooth', block: 'start' });
        });
      }
    } else {
      drawer.classList.add('hidden');
      drawer.classList.remove('is-opening');
      if (toggleLabel) {
        toggleLabel.textContent = 'Show Engineering Deep-Dive (5 Modules)';
      }
      if (shouldScroll) {
        engSection.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    }

    if (typeof window.refreshScrollSpy === 'function') {
      requestAnimationFrame(() => window.refreshScrollSpy(true));
    }
  }

  window.openEngineeringModule = (modKey = 'vision', scroll = true) => {
    setDrawerOpen(true, modKey, scroll);
  };

  toggleBtn?.addEventListener('click', () => {
    setDrawerOpen(!isOpen, !isOpen ? 'vision' : 'engineering', !isOpen);
  });

  closeBtn?.addEventListener('click', () => {
    setDrawerOpen(false, null, true);
  });

  backToTopBtn?.addEventListener('click', () => {
    const firstMod = document.getElementById('vision') || drawer;
    engLockUntil = performance.now() + 650;
    if (typeof window.setActiveEngSection === 'function') window.setActiveEngSection('vision');
    firstMod.scrollIntoView({ behavior: 'smooth', block: 'start' });
  });

  collapseBottomBtn?.addEventListener('click', () => {
    setDrawerOpen(false, null, true);
  });

  quickCards.forEach(card => {
    card.addEventListener('click', () => {
      const targetMod = card.dataset.openEng || 'vision';
      setDrawerOpen(true, targetMod, true);
    });
  });

  // Auto-open if page loads with an engineering hash
  const initialHash = window.location.hash;
  if (['#engineering', '#vision', '#electrical', '#safety', '#code', '#considerations'].includes(initialHash)) {
    const mod = initialHash === '#engineering' ? 'vision' : initialHash.substring(1);
    setTimeout(() => setDrawerOpen(true, mod, true), 180);
  }
}

/* ==========================================================================
   9. Dynamic Engineering Sub-Nav Scroll-Spy Sliding Highlight Pill (rAF-Throttled)
   ========================================================================== */
function initEngineeringScrollSpy() {
  const engNav = document.getElementById('eng-nav-pill');
  const selectionPill = document.getElementById('eng-selection-pill');
  const engLinks = Array.from(document.querySelectorAll('#eng-nav-pill .eng-pill-link'));
  const drawer = document.getElementById('engineering-drawer');

  if (!engNav || !selectionPill || engLinks.length === 0) return;

  const engSectionIds = engLinks.map(link => link.dataset.engSection).filter(Boolean);
  let currentActiveId = null;
  let ticking = false;

  function movePillToLink(activeLink, forceScrollCenter = false) {
    if (!activeLink) return;
    const left = activeLink.offsetLeft;
    const width = activeLink.offsetWidth;
    if (width === 0) return;

    selectionPill.style.width = `${width}px`;
    selectionPill.style.transform = `translate3d(${left}px, 0, 0)`;

    engLinks.forEach(link => {
      link.classList.toggle('active', link === activeLink);
    });

    if (forceScrollCenter && engNav.scrollWidth > engNav.clientWidth) {
      const pillCenter = left + width / 2;
      const navCenter = engNav.clientWidth / 2;
      engNav.scrollTo({
        left: Math.max(0, pillCenter - navCenter),
        behavior: 'smooth'
      });
    }
  }

  window.setActiveEngSection = (secId) => {
    const link = engLinks.find(l => l.dataset.engSection === secId);
    if (!link) return;
    currentActiveId = secId;
    movePillToLink(link, true);
  };

  function computeActiveEngineeringSection(force = false) {
    ticking = false;
    if (!drawer || drawer.classList.contains('hidden')) return;
    if (!force && performance.now() < engLockUntil) return;

    const triggerY = window.innerHeight * 0.34;
    let bestId = engSectionIds[0];

    for (const id of engSectionIds) {
      const el = document.getElementById(id);
      if (!el) continue;
      if (el.getBoundingClientRect().top <= triggerY) {
        bestId = id;
      }
    }

    const drawerRect = drawer.getBoundingClientRect();
    if (drawerRect.bottom <= window.innerHeight + 80) {
      bestId = engSectionIds[engSectionIds.length - 1];
    }

    if (!force && bestId === currentActiveId) return;
    currentActiveId = bestId;
    const activeLink = engLinks.find(l => l.dataset.engSection === currentActiveId) || engLinks[0];
    movePillToLink(activeLink, true);
  }

  window.refreshEngineeringScrollSpy = (force = true) => {
    requestAnimationFrame(() => computeActiveEngineeringSection(force));
  };

  engLinks.forEach(link => {
    link.addEventListener('click', (e) => {
      e.preventDefault();
      const secId = link.dataset.engSection;
      const targetEl = document.getElementById(secId);
      if (targetEl) {
        engLockUntil = performance.now() + 650;
        currentActiveId = secId;
        movePillToLink(link, true);
        targetEl.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    });
  });

  window.addEventListener('scroll', () => {
    if (!ticking) {
      ticking = true;
      requestAnimationFrame(() => computeActiveEngineeringSection(false));
    }
  }, { passive: true });
  window.addEventListener('resize', () => computeActiveEngineeringSection(true), { passive: true });

  requestAnimationFrame(() => computeActiveEngineeringSection(true));
}

/* ==========================================================================
   10. Dynamic Scroll-Spy Sliding Selection Pill in Top Navigation Bar
   ========================================================================== */
function initScrollSpyNav() {
  const navContainer = document.getElementById('top-nav-pill');
  const selectionPill = document.getElementById('nav-selection-pill');
  const navLinks = Array.from(document.querySelectorAll('#top-nav-pill .nav-pill-link'));

  if (!navContainer || !selectionPill || navLinks.length === 0) return;

  const sectionIds = navLinks.map(link => link.dataset.section).filter(Boolean);
  let currentActiveId = null;
  let ticking = false;

  function movePillToLink(activeLink) {
    if (!activeLink) return;
    const left = activeLink.offsetLeft;
    const width = activeLink.offsetWidth;
    if (width === 0) return;

    selectionPill.style.width = `${width}px`;
    selectionPill.style.transform = `translate3d(${left}px, 0, 0)`;

    selectionPill.classList.toggle('pill-engineering', activeLink.dataset.section === 'engineering');

    navLinks.forEach(link => {
      link.classList.toggle('active', link === activeLink);
    });
  }

  window.setActiveTopNavSection = (secId) => {
    const link = navLinks.find(l => l.dataset.section === secId);
    if (!link) return;
    currentActiveId = secId;
    movePillToLink(link);
  };

  function computeActiveSection(force = false) {
    ticking = false;
    if (!force && performance.now() < navLockUntil) return;

    const triggerY = window.innerHeight * 0.32;
    let bestId = sectionIds[0];

    for (const id of sectionIds) {
      const el = document.getElementById(id);
      if (!el) continue;
      if (el.getBoundingClientRect().top <= triggerY) {
        bestId = id;
      }
    }

    if (window.innerHeight + window.scrollY >= document.documentElement.scrollHeight - 30) {
      bestId = sectionIds[sectionIds.length - 1];
    }

    if (!force && bestId === currentActiveId) return;
    currentActiveId = bestId;
    const activeLink = navLinks.find(l => l.dataset.section === currentActiveId) || navLinks[0];
    movePillToLink(activeLink);
  }

  window.refreshScrollSpy = (force = true) => computeActiveSection(force);

  window.addEventListener('scroll', () => {
    if (!ticking) {
      ticking = true;
      requestAnimationFrame(() => computeActiveSection(false));
    }
  }, { passive: true });
  window.addEventListener('resize', () => computeActiveSection(true), { passive: true });

  // Reposition accurately on initial load & after web fonts finish loading
  requestAnimationFrame(() => computeActiveSection(true));
  if (document.fonts && document.fonts.ready) {
    document.fonts.ready.then(() => computeActiveSection(true));
  }
  setTimeout(() => computeActiveSection(true), 300);
}
