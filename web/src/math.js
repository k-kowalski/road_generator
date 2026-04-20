function clamp(value, minValue, maxValue) {
  return Math.min(maxValue, Math.max(minValue, value));
}

function subtract(left, right) {
  return [left[0] - right[0], left[1] - right[1], left[2] - right[2]];
}

function cross(left, right) {
  return [
    left[1] * right[2] - left[2] * right[1],
    left[2] * right[0] - left[0] * right[2],
    left[0] * right[1] - left[1] * right[0],
  ];
}

function dot(left, right) {
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

function normalize(value) {
  const length = Math.hypot(value[0], value[1], value[2]);
  if (length <= 1.0e-8) {
    return [0, 0, 0];
  }

  return [value[0] / length, value[1] / length, value[2] / length];
}

function transformVec4(matrix, vector) {
  return [
    matrix[0] * vector[0] + matrix[4] * vector[1] + matrix[8] * vector[2] + matrix[12] * vector[3],
    matrix[1] * vector[0] + matrix[5] * vector[1] + matrix[9] * vector[2] + matrix[13] * vector[3],
    matrix[2] * vector[0] + matrix[6] * vector[1] + matrix[10] * vector[2] + matrix[14] * vector[3],
    matrix[3] * vector[0] + matrix[7] * vector[1] + matrix[11] * vector[2] + matrix[15] * vector[3],
  ];
}

function multiplyMat4(left, right) {
  const result = new Float32Array(16);

  for (let column = 0; column < 4; column += 1) {
    for (let row = 0; row < 4; row += 1) {
      result[column * 4 + row] =
        left[0 * 4 + row] * right[column * 4 + 0] +
        left[1 * 4 + row] * right[column * 4 + 1] +
        left[2 * 4 + row] * right[column * 4 + 2] +
        left[3 * 4 + row] * right[column * 4 + 3];
    }
  }

  return result;
}

function perspectiveFovLH(fovYRadians, aspect, nearPlane, farPlane) {
  const f = 1.0 / Math.tan(fovYRadians * 0.5);
  const result = new Float32Array(16);
  result[0] = f / aspect;
  result[5] = f;
  result[10] = farPlane / (farPlane - nearPlane);
  result[11] = 1.0;
  result[14] = (-nearPlane * farPlane) / (farPlane - nearPlane);
  return result;
}

function lookAtLH(eye, target, up) {
  const zAxis = normalize(subtract(target, eye));
  const xAxis = normalize(cross(up, zAxis));
  const yAxis = cross(zAxis, xAxis);

  const result = new Float32Array(16);
  result[0] = xAxis[0];
  result[1] = yAxis[0];
  result[2] = zAxis[0];
  result[4] = xAxis[1];
  result[5] = yAxis[1];
  result[6] = zAxis[1];
  result[8] = xAxis[2];
  result[9] = yAxis[2];
  result[10] = zAxis[2];
  result[12] = -dot(xAxis, eye);
  result[13] = -dot(yAxis, eye);
  result[14] = -dot(zAxis, eye);
  result[15] = 1.0;
  return result;
}

function invertMat4(matrix) {
  const out = new Float32Array(16);

  const a00 = matrix[0];
  const a01 = matrix[1];
  const a02 = matrix[2];
  const a03 = matrix[3];
  const a10 = matrix[4];
  const a11 = matrix[5];
  const a12 = matrix[6];
  const a13 = matrix[7];
  const a20 = matrix[8];
  const a21 = matrix[9];
  const a22 = matrix[10];
  const a23 = matrix[11];
  const a30 = matrix[12];
  const a31 = matrix[13];
  const a32 = matrix[14];
  const a33 = matrix[15];

  const b00 = a00 * a11 - a01 * a10;
  const b01 = a00 * a12 - a02 * a10;
  const b02 = a00 * a13 - a03 * a10;
  const b03 = a01 * a12 - a02 * a11;
  const b04 = a01 * a13 - a03 * a11;
  const b05 = a02 * a13 - a03 * a12;
  const b06 = a20 * a31 - a21 * a30;
  const b07 = a20 * a32 - a22 * a30;
  const b08 = a20 * a33 - a23 * a30;
  const b09 = a21 * a32 - a22 * a31;
  const b10 = a21 * a33 - a23 * a31;
  const b11 = a22 * a33 - a23 * a32;

  const determinant =
    b00 * b11 -
    b01 * b10 +
    b02 * b09 +
    b03 * b08 -
    b04 * b07 +
    b05 * b06;

  if (Math.abs(determinant) <= 1.0e-8) {
    return null;
  }

  const inverseDeterminant = 1.0 / determinant;

  out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * inverseDeterminant;
  out[1] = (a02 * b10 - a01 * b11 - a03 * b09) * inverseDeterminant;
  out[2] = (a31 * b05 - a32 * b04 + a33 * b03) * inverseDeterminant;
  out[3] = (a22 * b04 - a21 * b05 - a23 * b03) * inverseDeterminant;
  out[4] = (a12 * b08 - a10 * b11 - a13 * b07) * inverseDeterminant;
  out[5] = (a00 * b11 - a02 * b08 + a03 * b07) * inverseDeterminant;
  out[6] = (a32 * b02 - a30 * b05 - a33 * b01) * inverseDeterminant;
  out[7] = (a20 * b05 - a22 * b02 + a23 * b01) * inverseDeterminant;
  out[8] = (a10 * b10 - a11 * b08 + a13 * b06) * inverseDeterminant;
  out[9] = (a01 * b08 - a00 * b10 - a03 * b06) * inverseDeterminant;
  out[10] = (a30 * b04 - a31 * b02 + a33 * b00) * inverseDeterminant;
  out[11] = (a21 * b02 - a20 * b04 - a23 * b00) * inverseDeterminant;
  out[12] = (a11 * b07 - a10 * b09 - a12 * b06) * inverseDeterminant;
  out[13] = (a00 * b09 - a01 * b07 + a02 * b06) * inverseDeterminant;
  out[14] = (a31 * b01 - a30 * b03 - a32 * b00) * inverseDeterminant;
  out[15] = (a20 * b03 - a21 * b01 + a22 * b00) * inverseDeterminant;
  return out;
}

function cameraEye(camera) {
  const cosPitch = Math.cos(camera.pitch);
  const sinPitch = Math.sin(camera.pitch);
  const cosYaw = Math.cos(camera.yaw);
  const sinYaw = Math.sin(camera.yaw);

  return [
    camera.target[0] + camera.distance * cosPitch * sinYaw,
    camera.target[1] + camera.distance * sinPitch,
    camera.target[2] + camera.distance * cosPitch * cosYaw,
  ];
}

function buildCameraMatrices(camera, aspect) {
  const eye = cameraEye(camera);
  const view = lookAtLH(eye, camera.target, [0, 1, 0]);
  const projection = perspectiveFovLH(Math.PI / 3, aspect, 0.1, 100.0);
  const viewProjection = multiplyMat4(projection, view);

  return {
    eye,
    view,
    projection,
    viewProjection,
  };
}

function screenPointToGroundPlane(screenX, screenY, width, height, camera) {
  const aspect = Math.max(width, 1) / Math.max(height, 1);
  const { viewProjection } = buildCameraMatrices(camera, aspect);
  const inverseViewProjection = invertMat4(viewProjection);
  if (inverseViewProjection === null) {
    return null;
  }

  const ndcX = (screenX / Math.max(width, 1)) * 2.0 - 1.0;
  const ndcY = 1.0 - (screenY / Math.max(height, 1)) * 2.0;

  const nearPoint = transformVec4(inverseViewProjection, [ndcX, ndcY, 0.0, 1.0]);
  const farPoint = transformVec4(inverseViewProjection, [ndcX, ndcY, 1.0, 1.0]);

  const near = [nearPoint[0] / nearPoint[3], nearPoint[1] / nearPoint[3], nearPoint[2] / nearPoint[3]];
  const far = [farPoint[0] / farPoint[3], farPoint[1] / farPoint[3], farPoint[2] / farPoint[3]];

  const rayY = far[1] - near[1];
  if (Math.abs(rayY) <= 1.0e-6) {
    return null;
  }

  const t = -near[1] / rayY;
  if (t < 0.0) {
    return null;
  }

  const point = [
    near[0] + (far[0] - near[0]) * t,
    0.0,
    near[2] + (far[2] - near[2]) * t,
  ];

  if (!Number.isFinite(point[0]) || !Number.isFinite(point[2])) {
    return null;
  }

  return point;
}

window.RoadDemoMath = {
  buildCameraMatrices,
  cameraEye,
  clamp,
  invertMat4,
  lookAtLH,
  multiplyMat4,
  perspectiveFovLH,
  screenPointToGroundPlane,
};
