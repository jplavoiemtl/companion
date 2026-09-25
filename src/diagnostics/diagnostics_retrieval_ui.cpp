#include "diagnostics_retrieval_ui.h"
#include "diagnostics_retrieval.h"
#include "diagnostics_network.h"
#include "../screen_memory/screen_memory.h"
#include "ui.h"
#include "calibration.h"
#include <WiFi.h>
#include "HWCDC.h"
#include <stdio.h>
#include <string.h>
extern HWCDC USBSerial;

namespace {
lv_obj_t* screen = nullptr;
lv_obj_t* stateLabel = nullptr;
lv_obj_t* urlLabel = nullptr;
lv_obj_t* stopButton = nullptr;
lv_obj_t* calibrationNotice = nullptr;
lv_obj_t* homeNotice = nullptr;
bool armed = false, pressed = false, consumed = false;
uint32_t pressedAt = 0, refreshedAt = 0, noticeAt = 0;
lv_obj_t* visibleNotice = nullptr;

const char* readable(const char* reason) {
  if (!strcmp(reason,"usb_power_required")) return "Connect USB power to download logs.";
  if (!strcmp(reason,"wifi_offline")) return "Connect your iPhone hotspot first.";
  if (!strcmp(reason,"image_busy") || !strcmp(reason,"live_busy") || !strcmp(reason,"display_pending")) return "Wait for the image or video to finish.";
  if (!strcmp(reason,"retrieval_busy")) return "Wait for the current download to finish.";
  if (!strcmp(reason,"calibration_busy")) return "Wait for calibration to finish.";
  if (!strcmp(reason,"logger_closing") || !strcmp(reason,"logger_unavailable")) return "Log storage is not ready.";
  if (!strcmp(reason,"interface_required") || !strcmp(reason,"interface_changed")) return "Connect using the iPhone hotspot.";
  if (!strcmp(reason,"idle_timeout")) return "Download mode closed after inactivity.";
  if (!strcmp(reason,"usb_power_lost")) return "Download mode closed: USB power removed.";
  if (!strcmp(reason,"panel_stop") || !strcmp(reason,"usb_command")) return "Download mode closed.";
  return "Download mode unavailable. Check log status.";
}
void setText(lv_obj_t* label, const char* text) {
  if (strcmp(lv_label_get_text(label),text)) lv_label_set_text(label,text);
}
void home() {
  _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Screen1_screen_init);
}
void showNotice(lv_obj_t* label, const char* reason) {
  if (visibleNotice) lv_obj_add_flag(visibleNotice,LV_OBJ_FLAG_HIDDEN);
  visibleNotice = label; noticeAt = lv_tick_get();
  setText(label,readable(reason)); lv_obj_clear_flag(label,LV_OBJ_FLAG_HIDDEN);
}
bool calibrationBusy() {
  const CalibState state = calibGetState();
  return state == CALIB_GRAVITY_SAMPLING || state == CALIB_FORWARD_SAMPLING || state == CALIB_READY_TO_COMPUTE;
}
void requestPanel() {
  if (calibrationBusy()) { showNotice(calibrationNotice,"calibration_busy"); return; }
  // Reopen an existing mode without running entry/tick or changing its origin/reason.
  if (logRetrievalView().phase == RetrievalPhase::Off) {
    const RetrievalEntry result = logRetrievalEnter(RetrievalOrigin::Panel);
    if (!result.accepted && strcmp(result.reason,"not_off")) {
      showNotice(calibrationNotice,result.reason); return;
    }
  }
  lv_disp_load_scr(screen);
  refreshedAt = lv_tick_get()-250; // Render the current state on the next background tick.
}
void stopEvent(lv_event_t* event) {
  activity_event_handler(event);
  const RetrievalPhase phase = logRetrievalView().phase;
  if (phase == RetrievalPhase::Starting || phase == RetrievalPhase::Active) {
    logRetrievalExit("panel_stop");
    setText(stateLabel,"Stopping...");
    lv_obj_add_state(stopButton,LV_STATE_DISABLED);
  }
}
lv_obj_t* labelAt(lv_obj_t* parent, int y, int width, const char* text) {
  lv_obj_t* label = lv_label_create(parent);
  lv_obj_set_width(label,width);
  lv_label_set_long_mode(label,LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(label,LV_TEXT_ALIGN_CENTER,0);
  lv_obj_align(label,LV_ALIGN_TOP_MID,0,y);
  lv_label_set_text(label,text);
  lv_obj_clear_flag(label,LV_OBJ_FLAG_CLICKABLE);
  return label;
}
lv_obj_t* makeNotice(lv_obj_t* parent, int width) {
  lv_obj_t* label = labelAt(parent,66,width,"");
  lv_obj_set_style_bg_color(label,lv_color_hex(0x202020),0);
  lv_obj_set_style_bg_opa(label,LV_OPA_COVER,0);
  lv_obj_set_style_text_color(label,lv_color_hex(0xffffff),0);
  lv_obj_add_flag(label,LV_OBJ_FLAG_HIDDEN);
  return label;
}
void memoryReport(const char* phase) {
  lv_mem_monitor_t memory;
  lv_mem_monitor(&memory);
  USBSerial.printf("[LOG UI MEM] phase=%s free=%u largest=%u fragmentation=%u\n",
    phase,unsigned(memory.free_size),unsigned(memory.free_biggest_size),unsigned(memory.frag_pct));
  diagnet::event("RETRIEVAL_UI_MEM","phase=%s free=%u largest=%u fragmentation=%u",
    phase,unsigned(memory.free_size),unsigned(memory.free_biggest_size),unsigned(memory.frag_pct));
}
} // namespace

void logRetrievalUiInit() {
  if (armed) return;
  memoryReport("before");
  screen = lv_obj_create(nullptr);
  lv_obj_clear_flag(screen,LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(screen,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_color(screen,lv_color_hex(0xffffff),0);
  lv_obj_set_style_text_font(screen,&lv_font_montserrat_20,0);
  const int width = lv_disp_get_hor_res(nullptr)-24;
  lv_obj_t* title = labelAt(screen,66,width,"Download logs");
  lv_obj_set_style_text_font(title,&lv_font_montserrat_24,0);
  stateLabel = labelAt(screen,105,width,"Starting...");
  lv_obj_set_style_text_font(stateLabel,&lv_font_montserrat_24,0);
  lv_obj_t* instructions = labelAt(screen,190,width,"On your iPhone");
  lv_obj_set_style_text_font(instructions,&lv_font_montserrat_24,0);
  urlLabel = labelAt(screen,230,width,"");
  lv_obj_set_style_text_font(urlLabel,&lv_font_montserrat_24,0);
  stopButton = lv_btn_create(screen);
  lv_obj_set_size(stopButton,width-24,48);
  lv_obj_align(stopButton,LV_ALIGN_BOTTOM_MID,0,-10);
  lv_obj_t* text = lv_label_create(stopButton);
  lv_label_set_text(text,"Stop and return"); lv_obj_center(text);
  lv_obj_add_event_cb(stopButton,stopEvent,LV_EVENT_CLICKED,nullptr);
  lv_obj_add_event_cb(screen,screenMemoryEventHandler,LV_EVENT_SCREEN_LOADED,nullptr);
  lv_obj_add_event_cb(screen,activity_event_handler,LV_EVENT_CLICKED,nullptr);
  calibrationNotice = makeNotice(ui_calibrationScreen,width);
  homeNotice = makeNotice(ui_Screen1,width);
  memoryReport("after");
  armed = true;
}

void logRetrievalUiEntryEvent(lv_event_t* event) {
  const lv_event_code_t code = lv_event_get_code(event);
  if (code == LV_EVENT_PRESSED) {
    pressed = true; consumed = false; pressedAt = lv_tick_get();
  } else if (code == LV_EVENT_PRESSING) {
    if (armed && pressed && !consumed && lv_tick_elaps(pressedAt) >= 1000) {
      consumed = true; requestPanel();
    }
  } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
    pressed = false; // Keep consumed through the ensuing CLICKED event.
  } else if (code == LV_EVENT_CLICKED) {
    if (consumed) return;
    diagnet::event("UI_ACTION","action=navigate button=button6 result=processed");
    home();
  }
}

void logRetrievalUiTick() {
  if (!armed) return;
  if (visibleNotice && lv_tick_elaps(noticeAt) >= 3000) {
    lv_obj_add_flag(visibleNotice,LV_OBJ_FLAG_HIDDEN); visibleNotice = nullptr;
  }
  if (lv_tick_elaps(refreshedAt) < 250) return;
  refreshedAt = lv_tick_get();
  if (lv_scr_act() != screen) return;
  const RetrievalView view = logRetrievalView();
  if (view.phase == RetrievalPhase::Off) {
    home(); showNotice(homeNotice,view.reason); return;
  }
  const char* status = view.releaseStuck ? "Still stopping. Wait or restart the unit." :
    view.phase == RetrievalPhase::Starting ? "Starting..." :
    view.phase == RetrievalPhase::Stopping ? "Stopping..." :
    view.linkUp ? "Ready" : "Hotspot disconnected - reconnect iPhone";
  setText(stateLabel,status);
  char url[32] = "";
  if (view.phase == RetrievalPhase::Active && view.linkUp) {
    const IPAddress ip = WiFi.localIP();
    snprintf(url,sizeof(url),"http://%u.%u.%u.%u/",unsigned(ip[0]),unsigned(ip[1]),unsigned(ip[2]),unsigned(ip[3]));
  }
  setText(urlLabel,url);
  const bool enabled = view.phase == RetrievalPhase::Starting || view.phase == RetrievalPhase::Active;
  if (enabled) lv_obj_clear_state(stopButton,LV_STATE_DISABLED);
  else lv_obj_add_state(stopButton,LV_STATE_DISABLED);
}

void logRetrievalUiPowerDown() {
  if (armed && lv_scr_act() == screen) home();
}
