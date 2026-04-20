(() => {
const createRoadDemoModuleFactory = window.createRoadDemoModule;
const {
  buildCameraMatrices,
  clamp,
  screenPointToGroundPlane,
} = window.RoadDemoMath;

const DEBUG_VERTEX_FLOATS = 6;
const RIBBON_VERTEX_FLOATS = 9;

const visibility = {
  showOriginalCurve: true,
  showSubdividedCurve: true,
  showRibbon: true,
  showRibbonWireframe: false,
};

const pressedKeys = new Set();

function makeDefaultCamera() {
  return {
    yaw: Math.PI,
    pitch: 0.48,
    distance: 8.4,
    target: [0.0, 0.15, 0.0],
  };
}

function copyFloat32(module, ptr, floatCount) {
  if (!ptr || floatCount <= 0) {
    return new Float32Array(0);
  }

  const source = new Float32Array(module.HEAPU8.buffer, ptr, floatCount);
  return new Float32Array(source);
}

function copyUint32(module, ptr, count) {
  if (!ptr || count <= 0) {
    return new Uint32Array(0);
  }

  const source = new Uint32Array(module.HEAPU8.buffer, ptr, count);
  return new Uint32Array(source);
}

function buildRibbonWireframeVertices(ribbonVertices, ribbonIndices) {
  if (ribbonVertices.length === 0 || ribbonIndices.length === 0) {
    return new Float32Array(0);
  }

  const floats = [];
  const pushVertex = (vertexIndex) => {
    const base = vertexIndex * RIBBON_VERTEX_FLOATS;
    floats.push(
      ribbonVertices[base + 0],
      ribbonVertices[base + 1],
      ribbonVertices[base + 2],
      ribbonVertices[base + 6],
      ribbonVertices[base + 7],
      ribbonVertices[base + 8],
    );
  };

  for (let i = 0; i + 2 < ribbonIndices.length; i += 3) {
    const a = ribbonIndices[i + 0];
    const b = ribbonIndices[i + 1];
    const c = ribbonIndices[i + 2];

    pushVertex(a);
    pushVertex(b);
    pushVertex(b);
    pushVertex(c);
    pushVertex(c);
    pushVertex(a);
  }

  return new Float32Array(floats);
}

function createRoadDemoBindings(module) {
  const context = module._road_demo_create();

  const ranges = (handle) => ({
    grid: {
      start: module._road_demo_grid_start(handle),
      count: module._road_demo_grid_count(handle),
    },
    roughCurve: {
      start: module._road_demo_rough_curve_start(handle),
      count: module._road_demo_rough_curve_count(handle),
    },
    roughControl: {
      start: module._road_demo_rough_control_start(handle),
      count: module._road_demo_rough_control_count(handle),
    },
    subdividedCurve: {
      start: module._road_demo_subdivided_curve_start(handle),
      count: module._road_demo_subdivided_curve_count(handle),
    },
    subdividedTangent: {
      start: module._road_demo_subdivided_tangent_start(handle),
      count: module._road_demo_subdivided_tangent_count(handle),
    },
  });

  return {
    appendPoint(x, y, z) {
      return module._road_demo_append_point(context, x, y, z) === 1;
    },
    finishCurve() {
      return module._road_demo_finish_curve(context) === 1;
    },
    rebuild() {
      module._road_demo_rebuild(context);
    },
    readScene() {
      const lineVertexCount = module._road_demo_line_vertex_count(context);
      const ribbonVertexCount = module._road_demo_ribbon_vertex_count(context);
      const ribbonIndexCount = module._road_demo_ribbon_index_count(context);

      const lineVertices = copyFloat32(
        module,
        module._road_demo_line_vertices(context),
        lineVertexCount * DEBUG_VERTEX_FLOATS,
      );
      const ribbonVertices = copyFloat32(
        module,
        module._road_demo_ribbon_vertices(context),
        ribbonVertexCount * RIBBON_VERTEX_FLOATS,
      );
      const ribbonIndices = copyUint32(
        module,
        module._road_demo_ribbon_indices(context),
        ribbonIndexCount,
      );

      return {
        lineVertices,
        ribbonVertices,
        ribbonIndices,
        ribbonWireframeVertices: buildRibbonWireframeVertices(ribbonVertices, ribbonIndices),
        ranges: ranges(context),
      };
    },
    destroy() {
      module._road_demo_destroy(context);
    },
  };
}

async function createRenderer(canvas) {
  if (!navigator.gpu) {
    throw new Error("WebGPU is not available in this browser.");
  }

  const adapter = await navigator.gpu.requestAdapter();
  if (!adapter) {
    throw new Error("Failed to acquire a WebGPU adapter.");
  }

  const device = await adapter.requestDevice();
  const context = canvas.getContext("webgpu");
  const format = navigator.gpu.getPreferredCanvasFormat();
  const lineShaderSource = window.RoadDemoShaders.lines;
  const ribbonShaderSource = window.RoadDemoShaders.ribbon;

  const uniformBuffer = device.createBuffer({
    label: "scene uniforms",
    size: 64,
    usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
  });

  const bindGroupLayout = device.createBindGroupLayout({
    entries: [
      {
        binding: 0,
        visibility: GPUShaderStage.VERTEX,
        buffer: { type: "uniform" },
      },
    ],
  });

  const pipelineLayout = device.createPipelineLayout({
    bindGroupLayouts: [bindGroupLayout],
  });

  const bindGroup = device.createBindGroup({
    layout: bindGroupLayout,
    entries: [
      {
        binding: 0,
        resource: { buffer: uniformBuffer },
      },
    ],
  });

  const linePipeline = device.createRenderPipeline({
    label: "debug lines",
    layout: pipelineLayout,
    vertex: {
      module: device.createShaderModule({ code: lineShaderSource }),
      entryPoint: "vs_main",
      buffers: [
        {
          arrayStride: DEBUG_VERTEX_FLOATS * 4,
          attributes: [
            { shaderLocation: 0, offset: 0, format: "float32x3" },
            { shaderLocation: 1, offset: 12, format: "float32x3" },
          ],
        },
      ],
    },
    fragment: {
      module: device.createShaderModule({ code: lineShaderSource }),
      entryPoint: "fs_main",
      targets: [{ format }],
    },
    primitive: {
      topology: "line-list",
    },
    depthStencil: {
      depthWriteEnabled: false,
      depthCompare: "less-equal",
      format: "depth24plus",
    },
  });

  const ribbonPipeline = device.createRenderPipeline({
    label: "road ribbon",
    layout: pipelineLayout,
    vertex: {
      module: device.createShaderModule({ code: ribbonShaderSource }),
      entryPoint: "vs_main",
      buffers: [
        {
          arrayStride: RIBBON_VERTEX_FLOATS * 4,
          attributes: [
            { shaderLocation: 0, offset: 0, format: "float32x3" },
            { shaderLocation: 1, offset: 12, format: "float32x2" },
            { shaderLocation: 2, offset: 20, format: "float32" },
          ],
        },
      ],
    },
    fragment: {
      module: device.createShaderModule({ code: ribbonShaderSource }),
      entryPoint: "fs_main",
      targets: [{ format }],
    },
    primitive: {
      topology: "triangle-list",
      cullMode: "none",
    },
    depthStencil: {
      depthWriteEnabled: true,
      depthCompare: "less-equal",
      format: "depth24plus",
    },
  });

  let depthTexture = null;
  let configuredWidth = 0;
  let configuredHeight = 0;

  const gpuBuffers = {
    lineVertexBuffer: null,
    lineVertexCount: 0,
    ribbonVertexBuffer: null,
    ribbonVertexCount: 0,
    ribbonIndexBuffer: null,
    ribbonIndexCount: 0,
    ribbonWireframeBuffer: null,
    ribbonWireframeVertexCount: 0,
  };

  const destroyBuffer = (buffer) => {
    if (buffer) {
      buffer.destroy();
    }
  };

  function resizeIfNeeded() {
    const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
    const nextWidth = Math.max(1, Math.floor(canvas.clientWidth * pixelRatio));
    const nextHeight = Math.max(1, Math.floor(canvas.clientHeight * pixelRatio));

    if (nextWidth === configuredWidth && nextHeight === configuredHeight) {
      return;
    }

    configuredWidth = nextWidth;
    configuredHeight = nextHeight;
    canvas.width = nextWidth;
    canvas.height = nextHeight;

    context.configure({
      device,
      format,
      alphaMode: "opaque",
    });

    if (depthTexture) {
      depthTexture.destroy();
    }

    depthTexture = device.createTexture({
      size: [configuredWidth, configuredHeight],
      format: "depth24plus",
      usage: GPUTextureUsage.RENDER_ATTACHMENT,
    });
  }

  function uploadScene(scene) {
    destroyBuffer(gpuBuffers.lineVertexBuffer);
    destroyBuffer(gpuBuffers.ribbonVertexBuffer);
    destroyBuffer(gpuBuffers.ribbonIndexBuffer);
    destroyBuffer(gpuBuffers.ribbonWireframeBuffer);

    gpuBuffers.lineVertexBuffer = null;
    gpuBuffers.ribbonVertexBuffer = null;
    gpuBuffers.ribbonIndexBuffer = null;
    gpuBuffers.ribbonWireframeBuffer = null;

    gpuBuffers.lineVertexCount = scene.lineVertices.length / DEBUG_VERTEX_FLOATS;
    gpuBuffers.ribbonVertexCount = scene.ribbonVertices.length / RIBBON_VERTEX_FLOATS;
    gpuBuffers.ribbonIndexCount = scene.ribbonIndices.length;
    gpuBuffers.ribbonWireframeVertexCount = scene.ribbonWireframeVertices.length / DEBUG_VERTEX_FLOATS;

    if (scene.lineVertices.byteLength > 0) {
      gpuBuffers.lineVertexBuffer = device.createBuffer({
        label: "debug line vertices",
        size: scene.lineVertices.byteLength,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
      });
      device.queue.writeBuffer(gpuBuffers.lineVertexBuffer, 0, scene.lineVertices);
    }

    if (scene.ribbonVertices.byteLength > 0) {
      gpuBuffers.ribbonVertexBuffer = device.createBuffer({
        label: "ribbon vertices",
        size: scene.ribbonVertices.byteLength,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
      });
      device.queue.writeBuffer(gpuBuffers.ribbonVertexBuffer, 0, scene.ribbonVertices);
    }

    if (scene.ribbonIndices.byteLength > 0) {
      gpuBuffers.ribbonIndexBuffer = device.createBuffer({
        label: "ribbon indices",
        size: scene.ribbonIndices.byteLength,
        usage: GPUBufferUsage.INDEX | GPUBufferUsage.COPY_DST,
      });
      device.queue.writeBuffer(gpuBuffers.ribbonIndexBuffer, 0, scene.ribbonIndices);
    }

    if (scene.ribbonWireframeVertices.byteLength > 0) {
      gpuBuffers.ribbonWireframeBuffer = device.createBuffer({
        label: "ribbon wireframe vertices",
        size: scene.ribbonWireframeVertices.byteLength,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
      });
      device.queue.writeBuffer(gpuBuffers.ribbonWireframeBuffer, 0, scene.ribbonWireframeVertices);
    }
  }

  function render(scene, camera) {
    resizeIfNeeded();

    const aspect = Math.max(configuredWidth, 1) / Math.max(configuredHeight, 1);
    const { viewProjection } = buildCameraMatrices(camera, aspect);
    device.queue.writeBuffer(uniformBuffer, 0, viewProjection);

    const commandEncoder = device.createCommandEncoder();
    const colorView = context.getCurrentTexture().createView();
    const depthView = depthTexture.createView();

    const pass = commandEncoder.beginRenderPass({
      colorAttachments: [
        {
          view: colorView,
          clearValue: { r: 0.06, g: 0.08, b: 0.12, a: 1.0 },
          loadOp: "clear",
          storeOp: "store",
        },
      ],
      depthStencilAttachment: {
        view: depthView,
        depthClearValue: 1.0,
        depthLoadOp: "clear",
        depthStoreOp: "store",
      },
    });

    pass.setBindGroup(0, bindGroup);

    const drawRangeIfAny = (range) => {
      if (range.count > 0) {
        pass.draw(range.count, 1, range.start, 0);
      }
    };

    if (
      visibility.showRibbon &&
      !visibility.showRibbonWireframe &&
      gpuBuffers.ribbonVertexBuffer &&
      gpuBuffers.ribbonIndexBuffer &&
      gpuBuffers.ribbonIndexCount > 0
    ) {
      pass.setPipeline(ribbonPipeline);
      pass.setVertexBuffer(0, gpuBuffers.ribbonVertexBuffer);
      pass.setIndexBuffer(gpuBuffers.ribbonIndexBuffer, "uint32");
      pass.drawIndexed(gpuBuffers.ribbonIndexCount);
    }

    pass.setPipeline(linePipeline);

    if (visibility.showRibbon && visibility.showRibbonWireframe && gpuBuffers.ribbonWireframeBuffer) {
      pass.setVertexBuffer(0, gpuBuffers.ribbonWireframeBuffer);
      pass.draw(gpuBuffers.ribbonWireframeVertexCount);
    }

    if (gpuBuffers.lineVertexBuffer) {
      pass.setVertexBuffer(0, gpuBuffers.lineVertexBuffer);
      drawRangeIfAny(scene.ranges.grid);

      if (visibility.showOriginalCurve) {
        drawRangeIfAny(scene.ranges.roughCurve);
        drawRangeIfAny(scene.ranges.roughControl);
      }

      if (visibility.showSubdividedCurve) {
        drawRangeIfAny(scene.ranges.subdividedCurve);
        drawRangeIfAny(scene.ranges.subdividedTangent);
      }
    }

    pass.end();
    device.queue.submit([commandEncoder.finish()]);
  }

  resizeIfNeeded();

  return {
    uploadScene,
    render,
    getWidth: () => configuredWidth,
    getHeight: () => configuredHeight,
  };
}

async function main() {
  const canvas = document.getElementById("scene");
  const status = document.getElementById("status");

  try {
    const module = await createRoadDemoModuleFactory();
    const demo = createRoadDemoBindings(module);
    const renderer = await createRenderer(canvas);

    const camera = makeDefaultCamera();
    let scene = null;
    let orbiting = false;
    let lastMouseX = 0;
    let lastMouseY = 0;
    let previousFrameTime = performance.now();

    const rebuildScene = () => {
      demo.rebuild();
      scene = demo.readScene();
      renderer.uploadScene(scene);
      status.textContent =
        `Layers: [1] rough ${visibility.showOriginalCurve ? "on" : "off"}  ` +
        `[2] subdivided ${visibility.showSubdividedCurve ? "on" : "off"}  ` +
        `[3] ribbon ${visibility.showRibbon ? "on" : "off"}  ` +
        `[4] wireframe ${visibility.showRibbonWireframe ? "on" : "off"}`;
    };

    rebuildScene();

    const localPointer = (event) => {
      const rect = canvas.getBoundingClientRect();
      return {
        x: event.clientX - rect.left,
        y: event.clientY - rect.top,
      };
    };

    canvas.addEventListener("contextmenu", (event) => event.preventDefault());

    canvas.addEventListener("mousedown", (event) => {
      const pointer = localPointer(event);

      if (event.button === 2) {
        orbiting = true;
        lastMouseX = pointer.x;
        lastMouseY = pointer.y;
        return;
      }

      if (event.button !== 0) {
        return;
      }

      const point = screenPointToGroundPlane(
        pointer.x * (renderer.getWidth() / Math.max(canvas.clientWidth, 1)),
        pointer.y * (renderer.getHeight() / Math.max(canvas.clientHeight, 1)),
        renderer.getWidth(),
        renderer.getHeight(),
        camera,
      );
      if (!point) {
        return;
      }

      if (demo.appendPoint(point[0], point[1], point[2])) {
        rebuildScene();
      }
    });

    window.addEventListener("mouseup", (event) => {
      if (event.button === 2) {
        orbiting = false;
      }
    });

    window.addEventListener("mousemove", (event) => {
      if (!orbiting) {
        return;
      }

      const pointer = localPointer(event);
      const dx = pointer.x - lastMouseX;
      const dy = pointer.y - lastMouseY;
      lastMouseX = pointer.x;
      lastMouseY = pointer.y;

      camera.yaw += dx * 0.01;
      camera.pitch = clamp(camera.pitch + dy * 0.01, -1.35, 1.35);
    });

    canvas.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        camera.distance *= event.deltaY < 0 ? 0.9 : 1.1;
        camera.distance = clamp(camera.distance, 2.0, 20.0);
      },
      { passive: false },
    );

    window.addEventListener("keydown", (event) => {
      const key = event.key.toLowerCase();
      if (key === "w" || key === "a" || key === "s" || key === "d") {
        pressedKeys.add(key);
      }

      if (event.repeat) {
        return;
      }

      switch (event.key) {
        case "Home":
          Object.assign(camera, makeDefaultCamera());
          break;
        case "1":
          visibility.showOriginalCurve = !visibility.showOriginalCurve;
          rebuildScene();
          break;
        case "2":
          visibility.showSubdividedCurve = !visibility.showSubdividedCurve;
          rebuildScene();
          break;
        case "3":
          visibility.showRibbon = !visibility.showRibbon;
          rebuildScene();
          break;
        case "4":
          visibility.showRibbonWireframe = !visibility.showRibbonWireframe;
          rebuildScene();
          break;
        case "Enter":
          if (demo.finishCurve()) {
            rebuildScene();
          }
          break;
        default:
          break;
      }
    });

    window.addEventListener("keyup", (event) => {
      const key = event.key.toLowerCase();
      if (key === "w" || key === "a" || key === "s" || key === "d") {
        pressedKeys.delete(key);
      }
    });

    window.addEventListener("blur", () => {
      pressedKeys.clear();
      orbiting = false;
    });

    const frame = (now) => {
      const deltaSeconds = Math.min((now - previousFrameTime) / 1000.0, 0.1);
      previousFrameTime = now;

      const panSpeed = Math.max(1.5, camera.distance * 0.9);
      const cosYaw = Math.cos(camera.yaw);
      const sinYaw = Math.sin(camera.yaw);

      let moveX = 0.0;
      let moveZ = 0.0;
      if (pressedKeys.has("w")) {
        moveX += -sinYaw;
        moveZ += -cosYaw;
      }
      if (pressedKeys.has("s")) {
        moveX -= -sinYaw;
        moveZ -= -cosYaw;
      }
      if (pressedKeys.has("d")) {
        moveX += -cosYaw;
        moveZ += sinYaw;
      }
      if (pressedKeys.has("a")) {
        moveX -= -cosYaw;
        moveZ -= sinYaw;
      }

      const moveLength = Math.hypot(moveX, moveZ);
      if (moveLength > 0.0) {
        const moveScale = (panSpeed * deltaSeconds) / moveLength;
        camera.target[0] += moveX * moveScale;
        camera.target[2] += moveZ * moveScale;
      }

      renderer.render(scene, camera);
      requestAnimationFrame(frame);
    };

    requestAnimationFrame(frame);
  } catch (error) {
    status.textContent = error instanceof Error ? error.message : String(error);
    status.classList.add("is-error");
  }
}

main();
})();
