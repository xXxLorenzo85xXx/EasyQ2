#include "easyq.h"
#include <zephyr/kernel.h>
#include "../../utilities/sh_defs.h"
#include "../../MAX86178/max86178.h"
#include "../../drivers/EASYQ/easyq_helper.h"

static int s_easyq_init();
static int s_easyq_powerOn();
static int s_easyq_powerOff();
static int s_easyq_startMeasurement();
static int s_easyq_stopMeasurement();
static int s_easyq_enableSensors(int en);
static int s_easyq_enableTimer(int tmr, int en);
static int s_easyq_updateTimerPeriod(int tmr, int period, int tmr_ps);
static void s_easyq_setBatteryPercentage(int battPer);
static int s_easyq_getBatteryPercentage();
static void s_easyq_setChargingStatus(bool en);
static bool s_easyq_getChargingStatus();
static void s_easyq_setStatusLedEnable(bool en);
static bool s_easyq_getStatusLedEnable();
static void s_easyq_setMeasurementEnable(bool en);
static bool s_easyq_getMesaurementEnable();
static void s_easyq_setDebugLogEnable(bool en);
static bool s_easyq_getDebugLogEnable();
static void s_easyq_setFlashLogEnable(bool en);
static bool s_easyq_getFlashLogEnable();
static void s_easyq_setTempSensorEnable(bool en);
static bool s_easyq_getTempSensorEnable();
static void s_easyq_setTempSensorFreq(int ind);
static int s_easyq_getTempSensorFreq();
static void s_easyq_setBleSentEnable(bool en);
static bool s_easyq_getBleSentEnable();
static void s_easyq_setBleSentEvt(int evt);
static int s_easyq_getBleSentEvt();
static void s_easyq_setStopCmd(int cmd);
static int s_easyq_getStopCmd();
static sensor_t* s_easyq_getSensor(int sensorNo);

static easyq_t easyq = {
    .init = s_easyq_init,
    .powerOn = s_easyq_powerOn,
    .powerOff = s_easyq_powerOff,
    .startMeasurement = s_easyq_startMeasurement,
    .stopMeasurement = s_easyq_stopMeasurement,
    .enableSensors = s_easyq_enableSensors,
    .enableTimer = s_easyq_enableTimer,
    .updateTimerPeriod = s_easyq_updateTimerPeriod,

    .setBatteryPercentage = s_easyq_setBatteryPercentage,
    .getBatteryPercentage = s_easyq_getBatteryPercentage,
    .setChargingStatus = s_easyq_setChargingStatus,
    .getChargingStatus = s_easyq_getChargingStatus,
    .setStatusLedEnable = s_easyq_setStatusLedEnable,
    .getStatusLedEnable = s_easyq_getStatusLedEnable,
    .setMeasurementEnable = s_easyq_setMeasurementEnable,
    .getMeasurementEnable = s_easyq_getMesaurementEnable,
    .setDebugLogEnable = s_easyq_setDebugLogEnable,
    .getDebugLogEnable = s_easyq_getDebugLogEnable,
    .setFlashLogEnable = s_easyq_setFlashLogEnable,
    .getFlashLogEnable = s_easyq_getFlashLogEnable,
    .setTempSensorEnable = s_easyq_setTempSensorEnable,
    .getTempSensorEnable = s_easyq_getTempSensorEnable,
    .setTempSensorFreq = s_easyq_setTempSensorFreq,
    .getTempSensorFreq = s_easyq_getTempSensorFreq,
    .setBleSentEnable = s_easyq_setBleSentEnable,
    .getBleSentEnable = s_easyq_getBleSentEnable,
    .setBleSentEvt = s_easyq_setBleSentEvt,
    .getBleSentEvt = s_easyq_getBleSentEvt,
    .setStopCmd = s_easyq_setStopCmd,
    .getStopCmd = s_easyq_getStopCmd,

    .getSensor = s_easyq_getSensor,
};


static int s_easyq_init()
{
    easyq.p_isInit = 1;
    // max86178_init(NULL);
     //easyq_ble_init();
    // easyq_comm_init();
    // easyq_peripheral_init();
    // easyq_api_init();
    // easyq_timers_init();
    // easyq_isr_init();

    //  mrd_sensor_init(&easyq.sensors[SH_ACC_SENSOR], SH_ACC_SENSOR);
      easyq_sensor_init(&easyq.sensors[SH_MAX86178], SH_MAX86178);
      		sensor_t * sensor_os64;


		sensor_os64 = mxm_sh_get_sensor_instance(SH_MAX86178);
    //  easyq_sensor_init(&easyq.sensors[SH_TEMP_SENSOR], SH_TEMP_SENSOR);

    return 0;
}

static int s_easyq_powerOn()
{
    return 0;
}
static int s_easyq_powerOff()
{
    return 0;
}
static int s_easyq_startMeasurement()
{
    return 0;
}
static int s_easyq_stopMeasurement()
{
    return 0;
}

static int s_easyq_enableSensors(int en)
{
   // easyq_sensors_enable(en);
    return 0;
}

static int s_easyq_enableTimer(int tmr, int en)
{

  //  easyq_timer_enable(tmr, en);

    return 0;
}

static int s_easyq_updateTimerPeriod(int tmr, int period, int tmr_ps)
{
  //  easyq_timer_period_update(tmr, period, tmr_ps);

    return 0;
}

static void s_easyq_setBatteryPercentage(int battPer)
{
    easyq.p_batteryPercentage = battPer;
}

static int s_easyq_getBatteryPercentage()
{
    return easyq.p_batteryPercentage;
}

static void s_easyq_setChargingStatus(bool en)
{
    easyq.p_chargingStatus = en;
}

static bool s_easyq_getChargingStatus()
{
    return easyq.p_chargingStatus;
}

static void s_easyq_setStatusLedEnable(bool en)
{
    easyq.p_statusLedEnable = en;
}

static bool s_easyq_getStatusLedEnable()
{
    return easyq.p_statusLedEnable;
}

static void s_easyq_setMeasurementEnable(bool en)
{
    easyq.p_measurementEnable = en;
}

static bool s_easyq_getMesaurementEnable()
{
    return easyq.p_measurementEnable;
}

static void s_easyq_setDebugLogEnable(bool en)
{

}

static bool s_easyq_getDebugLogEnable()
{
    return true;
}

static void s_easyq_setFlashLogEnable(bool en)
{
    easyq.p_flashLogEnable = en;
}

static bool s_easyq_getFlashLogEnable()
{
    return easyq.p_flashLogEnable;
}

static void s_easyq_setTempSensorEnable(bool en)
{
    easyq.p_tempSensorEnable = en;
}

static bool s_easyq_getTempSensorEnable()
{
    return easyq.p_tempSensorEnable;
}

static void s_easyq_setTempSensorFreq(int ind)
{
    easyq.p_tempSensorFreq = ind;
}

static int s_easyq_getTempSensorFreq()
{
    return easyq.p_tempSensorFreq;
}

static void s_easyq_setBleSentEnable(bool en)
{
    easyq.p_bleSentEnable = en;
}

static bool s_easyq_getBleSentEnable()
{
    return easyq.p_bleSentEnable;
}

static void s_easyq_setBleSentEvt(int evt)
{
    easyq.e_bleSentEvt = evt;
}

static int s_easyq_getBleSentEvt()
{
    return easyq.e_bleSentEvt;
}

static void s_easyq_setStopCmd(int cmd)
{
    easyq.p_stopCmd = cmd;
}

static int s_easyq_getStopCmd()
{
    return easyq.p_stopCmd;
}

static sensor_t* s_easyq_getSensor(int sensorNo)
{
    return easyq.sensors[sensorNo];
}

easyq_t* geteasyq()
{
    return &easyq;
}