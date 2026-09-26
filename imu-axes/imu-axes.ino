#include <M5Unified.h>

// 三轴姿态仪(世界固定视角)
// 红=世界X 绿=世界Y 蓝=世界Z(指向真实天空) —— 三轴固定在空间中
// 转动板子 = 移动观察视角, 屏幕上三轴实时反向转动
// 棕色圆盘=地平面(始终与蓝轴垂直), 姿态解算: 旋转矩阵+陀螺积分+加速度修正
// BtnA = 航向归零(让世界X对准板子当前朝向)

static constexpr float TAU_S    = 0.5f;   // 加速度修正时间常数(s)
static constexpr float D2R      = 0.017453293f;
static constexpr float R2D      = 57.29577951f;
static constexpr float AXIS_LEN = 44.0f;  // 轴长(像素)
static constexpr int   HUD_H    = 30;     // 底部数字栏高度

static M5Canvas spr(&M5.Display);

// 机体三轴在世界系中的方向(即旋转矩阵的三列), 世界系: Z=天
static float bx[3] = {1, 0, 0};
static float by[3] = {0, 1, 0};
static float bz[3] = {0, 0, 1};
static bool inited = false;
static uint32_t last_us = 0, last_draw = 0;

static inline float vdot(const float* a, const float* b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static inline void vcross(const float* a, const float* b, float* o) {
  o[0] = a[1]*b[2] - a[2]*b[1];
  o[1] = a[2]*b[0] - a[0]*b[2];
  o[2] = a[0]*b[1] - a[1]*b[0];
}
static inline void vnorm(float* v) {
  float n = sqrtf(vdot(v, v));
  if (n > 1e-6f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}
// v += w × v * k   (w为小旋转向量, 方向=转轴, 模长=转角rad)
static inline void vrot(float* v, const float* w, float k) {
  float c[3];
  vcross(w, v, c);
  v[0] += c[0]*k; v[1] += c[1]*k; v[2] += c[2]*k;
}

// 世界系向量 -> 机体系 (R^T * v)
static void worldToBody(const float* w, float* b) {
  b[0] = bx[0]*w[0] + bx[1]*w[1] + bx[2]*w[2];
  b[1] = by[0]*w[0] + by[1]*w[1] + by[2]*w[2];
  b[2] = bz[0]*w[0] + bz[1]*w[1] + bz[2]*w[2];
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(160);
  M5.Display.fillScreen(TFT_BLACK);
  spr.setColorDepth(16);
  spr.createSprite(M5.Display.width(), M5.Display.height());
}

void loop() {
  M5.update();
  uint32_t now = millis();
  uint32_t now_us = micros();
  float dt = last_us ? (now_us - last_us) * 1e-6f : 0.008f;
  last_us = now_us;
  if (dt > 0.05f) dt = 0.05f;

  float ax, ay, az, gx, gy, gz;
  M5.Imu.getAccel(&ax, &ay, &az);
  M5.Imu.getGyro(&gx, &gy, &gz);

  // --- 陀螺积分: 角速度从机体系变换到世界系, 再旋转三个轴 ---
  float wb[3] = {gx*D2R, gy*D2R, gz*D2R};
  float ww[3] = {
    bx[0]*wb[0] + by[0]*wb[1] + bz[0]*wb[2],
    bx[1]*wb[0] + by[1]*wb[1] + bz[1]*wb[2],
    bx[2]*wb[0] + by[2]*wb[1] + bz[2]*wb[2],
  };
  vrot(bx, ww, dt);
  vrot(by, ww, dt);
  vrot(bz, ww, dt);

  // --- 加速度计倾角修正(仅准静态, 此时比力方向=天) ---
  float amag = sqrtf(ax*ax + ay*ay + az*az);
  if (amag > 0.8f && amag < 1.2f) {
    float ab[3] = {ax/amag, ay/amag, az/amag};
    if (!inited) {
      // 首次直接对齐: R的第三行 = 机体测到的"天"
      float t[3] = {0, 0, 1};
      if (fabsf(ab[2]) >= 0.9f) { t[0] = 1; t[2] = 0; }
      // r0 = normalize(t × ab); r1 = ab × r0; r2 = ab
      float r0[3], r1[3];
      vcross(t, ab, r0); vnorm(r0);
      vcross(ab, r0, r1);
      bx[0]=r0[0]; bx[1]=r1[0]; bx[2]=ab[0];
      by[0]=r0[1]; by[1]=r1[1]; by[2]=ab[1];
      bz[0]=r0[2]; bz[1]=r1[2]; bz[2]=ab[2];
      inited = true;
    } else {
      // 误差 = 估计的天 × 真正的天(世界系), 按互补系数回拉
      float up_est[3] = {
        bx[0]*ab[0] + by[0]*ab[1] + bz[0]*ab[2],
        bx[1]*ab[0] + by[1]*ab[1] + bz[1]*ab[2],
        bx[2]*ab[0] + by[2]*ab[1] + bz[2]*ab[2],
      };
      float up_true[3] = {0, 0, 1};
      float e[3];
      vcross(up_est, up_true, e);
      float k = dt / (TAU_S + dt);
      vrot(bx, e, k);
      vrot(by, e, k);
      vrot(bz, e, k);
    }
  }

  // --- 正交归一化(消除积分漂移导致的畸变) ---
  vnorm(bx);
  float d = vdot(bx, by);
  by[0] -= d*bx[0]; by[1] -= d*bx[1]; by[2] -= d*bx[2];
  vnorm(by);
  vcross(bx, by, bz);

  // --- BtnA: 航向归零(绕世界Z轴旋转, 使红轴水平投影指向世界X) ---
  if (M5.BtnA.wasPressed()) {
    float psi = atan2f(bx[1], bx[0]);
    float c = cosf(psi), s = sinf(psi);
    float* cols[3] = {bx, by, bz};
    for (auto v : cols) {
      float x = v[0], y = v[1];
      v[0] =  x*c + y*s;
      v[1] = -x*s + y*c;
    }
  }

  if (now - last_draw >= 33) { last_draw = now; draw(); }
  delay(4);
}

// 机体系 -> 屏幕投影(斜视): 屏幕右=+Y, 屏幕下=+X, +Z(出屏)带斜移呈现立体感
static void project(const float* v, float& sx, float& sy) {
  const float cx = spr.width() * 0.5f;
  const float cy = (spr.height() - HUD_H) * 0.52f;
  sx = cx + AXIS_LEN * (v[1] + 0.38f * v[2]);
  sy = cy + AXIS_LEN * (v[0] - 0.38f * v[2]);
}

static void line3(const float* a, const float* b, float wdt, uint32_t color) {
  float ax_, ay_, bx_, by_;
  project(a, ax_, ay_);
  project(b, bx_, by_);
  spr.drawWideLine((int)ax_, (int)ay_, (int)bx_, (int)by_, wdt, color);
}

// 画一根轴: 正向亮线+箭头+标签, 反向短暗线
static void drawAxis(const float* v, uint32_t bright, uint32_t dim, const char* label) {
  float o[3] = {0, 0, 0};
  float neg[3] = {-v[0]*0.30f, -v[1]*0.30f, -v[2]*0.30f};
  line3(o, neg, 1.0f, dim);
  line3(o, v, 2.2f, bright);

  // 箭头(屏幕空间)
  float ox, oy, tx, ty;
  project(o, ox, oy);
  project(v, tx, ty);
  float dx = tx - ox, dy = ty - oy;
  float n = sqrtf(dx*dx + dy*dy);
  if (n > 1e-3f) {
    dx /= n; dy /= n;
    float px = -dy, py = dx;
    spr.drawWideLine((int)tx, (int)ty, (int)(tx - dx*8 + px*4), (int)(ty - dy*8 + py*4), 1.2f, bright);
    spr.drawWideLine((int)tx, (int)ty, (int)(tx - dx*8 - px*4), (int)(ty - dy*8 - py*4), 1.2f, bright);
  }

  float tip[3] = {v[0]*1.24f, v[1]*1.24f, v[2]*1.24f};
  float lx, ly;
  project(tip, lx, ly);
  spr.setFont(&lgfx::fonts::efontCN_10);
  spr.setTextColor(bright);
  spr.setTextDatum(middle_center);
  spr.drawString(label, lx, ly);
}

void draw() {
  auto& d = spr;
  const int w = d.width(), h = d.height();
  d.fillScreen(TFT_BLACK);

  float cx0, cy0;
  float o[3] = {0, 0, 0};
  project(o, cx0, cy0);

  // --- 地平面圆盘(世界Z=0平面, 转到机体系画) ---
  const int N = 36;
  const float RG = 1.05f;
  float px_[N + 1], py_[N + 1];
  for (int i = 0; i <= N; i++) {
    float a = (i % N) * 2.0f * M_PI / N;
    float pw[3] = {RG*cosf(a), RG*sinf(a), 0};
    float pb[3];
    worldToBody(pw, pb);
    project(pb, px_[i], py_[i]);
  }
  for (int i = 0; i < N; i++) {
    d.setColor(52, 36, 22);
    d.fillTriangle((int)cx0, (int)cy0, (int)px_[i], (int)py_[i],
                   (int)px_[i + 1], (int)py_[i + 1]);
  }
  for (int i = 0; i < N; i++) {
    spr.drawWideLine((int)px_[i], (int)py_[i], (int)px_[i + 1], (int)py_[i + 1], 0.6f, 0x4A5058u);
  }

  // --- 地面十字(世界X/Y方向) ---
  float wxp[3], wxm[3] = {-1, 0, 0}, wyp[3] = {0, 1, 0}, wym[3] = {0, -1, 0};
  wxp[0] = 1; wxp[1] = 0; wxp[2] = 0;
  float t1[3], t2[3];
  worldToBody(wxm, t1); worldToBody(wxp, t2);
  line3(t1, t2, 0.5f, 0x5A616Au);
  worldToBody(wym, t1); worldToBody(wyp, t2);
  line3(t1, t2, 0.5f, 0x5A616Au);

  // --- 世界三轴在机体系中的方向(固定在空间中, 随板子转动而实时变化) ---
  static const float ex[3] = {1, 0, 0};
  static const float ey[3] = {0, 1, 0};
  static const float ez[3] = {0, 0, 1};
  float wx_b[3], wy_b[3], wz_b[3];
  worldToBody(ex, wx_b);
  worldToBody(ey, wy_b);
  worldToBody(ez, wz_b);
  drawAxis(wx_b, 0xFF4646u, 0x6E2323u, "X");
  drawAxis(wy_b, 0x50E65Au, 0x236428u, "Y");
  drawAxis(wz_b, 0x5A8CFFu, 0x283C6Eu, "Z");

  // 蓝轴即真实天向, 加个"天"标注
  float lab[3] = {wz_b[0]*1.42f, wz_b[1]*1.42f, wz_b[2]*1.42f};
  float lx, ly;
  project(lab, lx, ly);
  d.setFont(&lgfx::fonts::efontCN_10);
  d.setTextColor(0x9AB8FFu);
  d.setTextDatum(middle_center);
  d.drawString("天", lx, ly);

  // 顶栏
  d.setFont(&lgfx::fonts::efontCN_10);
  d.setTextColor(TFT_LIGHTGREY);
  d.setTextDatum(top_left);
  d.drawString("三轴姿态", 2, 2);
  d.setTextDatum(top_right);
  d.drawString("A:航向归零", w - 2, 2);

  // --- 底部数字栏 ---
  // 实测轴向: X=屏幕右, Y=屏幕下(长边), Z=出屏
  // 俯仰 = 长边(Y)的抬头角; 横滚 = 短边(X)的右倾角
  float pitch = asinf(fmaxf(-1.f, fminf(1.f, -by[2]))) * R2D;
  float roll  = atan2f(-bx[2], bz[2]) * R2D;
  float yaw   = atan2f(bx[1], bx[0]) * R2D;
  d.setColor(12, 14, 18);
  d.fillRect(0, h - HUD_H, w, HUD_H);
  d.setColor(70, 76, 86);
  d.drawFastHLine(0, h - HUD_H, w);
  d.setFont(&lgfx::fonts::efontCN_10);
  d.setTextColor(TFT_LIGHTGREY);
  d.setTextDatum(top_center);
  d.drawString("俯仰", 24, h - HUD_H + 1);
  d.drawString("横滚", 68, h - HUD_H + 1);
  d.drawString("偏航", 112, h - HUD_H + 1);
  d.setTextColor(TFT_WHITE);
  char b[12];
  snprintf(b, sizeof(b), "%+4.0f", pitch); d.drawString(b, 24, h - HUD_H + 14);
  snprintf(b, sizeof(b), "%+4.0f", roll);  d.drawString(b, 68, h - HUD_H + 14);
  snprintf(b, sizeof(b), "%+4.0f", yaw);   d.drawString(b, 112, h - HUD_H + 14);

  spr.pushSprite(0, 0);
}
