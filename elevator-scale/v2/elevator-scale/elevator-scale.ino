#include <M5Unified.h>

// 电梯超重失重演示:加速度计测比力 |a|
// 大读数:运动加速度 a=(|a|-1g)·g0 (静止0 / 向上加速为正 / 自由落体-9.8)
// 底部:抛高 h=gT²/8 (T=失重段时长+80ms检测补偿,出手接手同高时成立)
// BtnA:冻结/继续画面 (冻结时白框+提示,再按恢复)
// 双缓冲(Sprite)绘制避免整屏清屏闪烁;历史曲线按状态着色

static constexpr float GAUGE_MAX      = 2.0f;
static constexpr float G_STD          = 9.81f;
static constexpr float G_LO           = 0.85f;
static constexpr float G_HI           = 1.15f;
static constexpr float FF_THRESH      = 0.20f;
static constexpr int   HIST_N         = 160;
static constexpr uint32_t FF_COMP_MS  = 80;   // 失重入段滞后~100ms 减 出段滞后~25ms
static constexpr float   MIN_T_S      = 0.25f; // 短于此时长的失重不算一次抛掷

enum State { ST_FREEFALL, ST_LIGHT, ST_NORMAL, ST_HEAVY };

static M5Canvas spr(&M5.Display);
static float   hist[HIST_N];
static uint8_t hist_st[HIST_N];
static int     hist_idx   = 0;
static float   r_smooth   = 1.0f;
static uint32_t low_ms    = 0;
static bool    freefall   = false;
static uint32_t ff_start  = 0;
static float   impact_peak = 0;
static uint32_t banner_until = 0;
static float   banner_peak = 0;
static uint32_t last_draw  = 0;
static uint32_t last_push  = 0;
static State   cur_state   = ST_NORMAL;
static bool    frozen      = false;
static bool    has_height  = false;
static float   last_h      = 0;
static float   last_T      = 0;

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t stateColor(State s) {
  switch (s) {
    case ST_FREEFALL: return TFT_RED;
    case ST_LIGHT:    return TFT_CYAN;
    case ST_NORMAL:   return TFT_DARKGREEN;
    default:          return TFT_ORANGE;
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(160);
  M5.Display.fillScreen(TFT_BLACK);
  spr.setColorDepth(16);
  spr.createSprite(M5.Display.width(), M5.Display.height());
  for (int i = 0; i < HIST_N; i++) { hist[i] = 1.0f; hist_st[i] = ST_NORMAL; }
}

void drawFreezeOverlay() {
  auto& d = M5.Display;
  const int w = d.width(), h = d.height();
  d.drawRect(0, 0, w, h, TFT_WHITE);
  d.drawRect(1, 1, w - 2, h - 2, TFT_WHITE);
  const int y = (int)(h * 0.40);
  d.fillRect(0, y - 10, w, 20, TFT_BLACK);
  d.setFont(&lgfx::fonts::efontCN_12);
  d.setTextDatum(middle_center);
  d.setTextColor(TFT_WHITE);
  d.drawString("冻结 - 按A继续", w / 2, y);
}

void loop() {
  M5.update();
  uint32_t now = millis();

  if (M5.BtnA.wasPressed()) {
    frozen = !frozen;
    if (frozen) drawFreezeOverlay();
  }
  if (frozen) { delay(8); return; }

  float ax, ay, az;
  M5.Imu.getAccel(&ax, &ay, &az);
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  r_smooth += (mag - r_smooth) * 0.35f;

  if (!freefall) {
    low_ms = (r_smooth < FF_THRESH) ? low_ms + 8 : 0;
    if (low_ms >= 60) {
      freefall = true;
      ff_start = now;
      impact_peak = 0;
    }
  } else {
    if (r_smooth > impact_peak) impact_peak = r_smooth;
    if (r_smooth > 0.85f && now - ff_start > 150) {
      freefall = false;
      if (impact_peak > 2.0f) {
        banner_peak = impact_peak;
        banner_until = now + 1600;
      }
      float T = (now - ff_start + FF_COMP_MS) * 0.001f;
      if (T >= MIN_T_S) {
        last_T = T;
        last_h = G_STD * T * T / 8.0f;
        has_height = true;
      }
    }
  }

  if (freefall)              cur_state = ST_FREEFALL;
  else if (r_smooth < G_LO)  cur_state = ST_LIGHT;
  else if (r_smooth <= G_HI) cur_state = ST_NORMAL;
  else                       cur_state = ST_HEAVY;

  if (now - last_push >= 16) {
    last_push = now;
    hist[hist_idx] = r_smooth;
    hist_st[hist_idx] = (uint8_t)cur_state;
    hist_idx = (hist_idx + 1) % HIST_N;
  }

  if (now - last_draw >= 33) {
    last_draw = now;
    draw(now);
  }
  delay(8);
}

void draw(uint32_t now) {
  auto& d = spr;
  const int w = d.width();
  const int h = d.height();

  d.fillScreen(TFT_BLACK);

  // 状态条
  const int band_h = h * 0.13;
  bool flash = (cur_state == ST_FREEFALL) && ((now / 150) % 2 == 0);
  d.fillRect(0, 0, w, band_h, flash ? TFT_WHITE : stateColor(cur_state));
  d.setFont(&lgfx::fonts::efontCN_24);
  d.setTextSize(1);
  d.setTextDatum(middle_center);
  d.setTextColor(flash ? TFT_RED : TFT_BLACK);
  if (now < banner_until) {
    char buf[32];
    snprintf(buf, sizeof(buf), "落地峰值 %.1f g", banner_peak);
    d.fillRect(0, 0, w, band_h, TFT_ORANGE);
    d.setTextColor(TFT_BLACK);
    d.drawString(buf, w / 2, band_h / 2);
  } else {
    const char* label;
    switch (cur_state) {
      case ST_FREEFALL: label = "完全失重 N=0"; break;
      case ST_LIGHT:    label = "失重 N<mg";    break;
      case ST_NORMAL:   label = "正常 N=mg";    break;
      default:          label = "超重 N>mg";    break;
    }
    d.drawString(label, w / 2, band_h / 2);
  }

  // 大读数:运动加速度 a=(|a|-1g)·g0
  const int y_big0 = band_h, y_big1 = h * 0.36;
  float a_kin = (r_smooth - 1.0f) * G_STD;
  char num[16];
  snprintf(num, sizeof(num), (fabsf(a_kin) < 10.0f) ? "%+.1f" : "%+.0f", a_kin);
  d.setTextColor(TFT_WHITE);
  d.setFont(&lgfx::fonts::efontCN_24);
  d.setTextSize(2);
  int numW = d.textWidth(num);
  d.setFont(&lgfx::fonts::efontCN_16);
  d.setTextSize(1);
  int unitW = d.textWidth("m/s2");
  int x0 = (w - numW - 4 - unitW) / 2;
  const int ymid = (y_big0 + y_big1) / 2;
  d.setFont(&lgfx::fonts::efontCN_24);
  d.setTextSize(2);
  d.setTextDatum(middle_left);
  d.drawString(num, x0, ymid);
  d.setFont(&lgfx::fonts::efontCN_16);
  d.setTextSize(1);
  d.drawString("m/s2", x0 + numW + 4, ymid);

  // 副读数:比力
  char sub[48];
  snprintf(sub, sizeof(sub), "比力 |a|=%.2fg", r_smooth);
  d.setFont(&lgfx::fonts::efontCN_12);
  d.setTextDatum(middle_center);
  d.setTextColor(TFT_LIGHTGREY);
  d.drawString(sub, w / 2, h * 0.40);

  // 仪表条 0..2.5g
  const int gx = w * 0.05, gw = w * 0.90;
  const int gy = h * 0.46, gh = h * 0.055;
  d.drawRect(gx, gy, gw, gh, TFT_DARKGREY);
  int fillw = (int)(gw * clampf(r_smooth, 0, GAUGE_MAX) / GAUGE_MAX);
  if (fillw > 2) d.fillRect(gx + 1, gy + 1, fillw - 2, gh - 2, stateColor(cur_state));
  const int tx = gx + (int)(gw / GAUGE_MAX);
  d.drawLine(tx, gy - 3, tx, gy + gh + 3, TFT_WHITE);
  d.setFont(&lgfx::fonts::efontCN_10);
  d.setTextColor(TFT_LIGHTGREY);
  d.setTextDatum(top_left);   d.drawString("0", gx, gy + gh + 4);
  d.setTextDatum(top_center); d.drawString("1g", tx, gy + gh + 4);
  d.setTextDatum(top_right);  d.drawString("2g", gx + gw, gy + gh + 4);

  // a-t 滚动曲线,按状态着色
  const int cy = h * 0.58, ch = h * 0.33;
  const int y1g = cy + ch - (int)(ch / GAUGE_MAX);
  for (int x = 0; x < w; x += 10) d.drawLine(x, y1g, x + 5, y1g, TFT_DARKGREY);
  float step = (float)w / (HIST_N - 1);
  int px = 0, py = 0;
  for (int i = 0; i < HIST_N; i++) {
    int idx = (hist_idx + i) % HIST_N;
    int x = (int)(i * step);
    int y = cy + ch - (int)(ch * clampf(hist[idx], 0, GAUGE_MAX) / GAUGE_MAX);
    if (i > 0) d.drawLine(px, py, x, y, stateColor((State)hist_st[idx]));
    px = x; py = y;
  }

  // 底部:抛高
  d.setFont(&lgfx::fonts::efontCN_12);
  d.setTextDatum(middle_center);
  d.setTextColor(TFT_WHITE);
  if (has_height) {
    char hb[40];
    snprintf(hb, sizeof(hb), "抛高 %.2fm T=%.2fs", last_h, last_T);
    d.drawString(hb, w / 2, h * 0.95);
  } else {
    d.setTextColor(TFT_LIGHTGREY);
    d.drawString("抛高:上抛来测", w / 2, h * 0.95);
  }

  spr.pushSprite(0, 0);
}
