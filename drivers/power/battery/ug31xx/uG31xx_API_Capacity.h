/**
 * @filename  uG31xx_API_Capacity.h
 *
 *  Header of uG31xx capacity algorithm
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 2 $
 */

typedef char            _cap_bool_;
typedef unsigned char   _cap_u8_;
typedef signed char     _cap_s8_;
typedef unsigned short  _cap_u16_;
typedef signed short    _cap_s16_;
typedef unsigned long   _cap_u32_;
typedef signed long     _cap_s32_;

#define CAP_TRUE                (1)
#define CAP_FALSE               (0)

#define CAP_FC_RELEASE_RSOC     (99)
#define SOC_LOCK_TIME_STEP      (10000) /// 15 sec
#define SOC_CHG_LOCK_TIME       (120000) /// 180 sec
#define SOC_DSG_LOCK_TIME       (120000) /// 120 sec

#define CHARGE_VOLTAGE_CONST      (4)

#ifdef  UG31XX_SHELL_ALGORITHM

typedef struct CellParameterST {
} ALIGNED_ATTRIBUTE CellParameterType, CELL_PARAMETER;

typedef struct CellTableST {
} ALIGNED_ATTRIBUTE CellTableType, CELL_TABLE;

typedef struct MeasDataST {
} ALIGNED_ATTRIBUTE MeasDataType;

extern CellParameterType ggbParameter;
extern CellTableType cellTable;
extern MeasDataType measData;

#endif  ///< end of UG31XX_SHELL_ALGORITHM

typedef struct CapacityDataST {

  /// [AT-PM] : Data from GGB file ; 01/25/2013
  CELL_PARAMETER *ggbParameter;
  CELL_TABLE *ggbTable;
  
  /// [AT-PM] : Measurement data ; 01/25/2013
  MeasDataType *measurement;
  
  _cap_u8_ *encriptTable;
  /// [FC] : encript backup in order to compare the difference ; 05/30/2013
  _cap_u8_ *encriptBuf;
  _cap_u8_ tableSize;

  /// [AT-PM] : Capacity information ; 01/25/2013
  _cap_u16_ rm;
  _cap_u16_ fcc;
  _cap_u16_ fccBackup;
  _cap_u16_ fccBeforeChg;
  _cap_u8_ rsoc;
  _cap_u8_ predictRsoc;
  _cap_bool_ fcSts;
  
  /// [AT-PM] : Capacity operation variables ; 01/25/2013
  _cap_u32_ status;

  _cap_u32_ selfDsgMilliSec;
  _cap_u8_ selfDsgSec;
  _cap_u8_ selfDsgMin;
  _cap_u8_ selfDsgHour;
  _cap_u8_ selfDsgResidual;

  _cap_u32_ standbyDsgResidual;

  _cap_u8_ lastRsoc;

  _cap_u32_ tpTime;

  _cap_s32_ dsgCharge;
  _cap_s32_ dsgChargeStart;
  _cap_u32_ dsgChargeTime;
  _cap_s32_ preDsgCharge;

  _cap_u8_ tableUpdateIdx;
  _cap_u32_ tableUpdateDisqTime;

  _cap_s8_ parseRMResidual;

  _cap_s16_ reverseCap;
  _cap_u8_ avgCRate;  
  _cap_u16_ avgVoltage;
  _cap_s16_ avgTemperature;

  _cap_s16_ ccRecord[SOV_NUMS];
  _cap_u8_ chgPredictSOCStep;
  _cap_u32_ lockTimeChg;
  _cap_u32_ lockTimeDsg;
  _cap_u8_ transferStateToChg;
  _cap_u16_ startChgVolt;
  _cap_u8_ startChgRsoc;
  _cap_u16_ avgRM;
  _cap_s16_ lastCVDeltaChgCurr;
  _cap_u8_ cvCheckCnt;

  _cap_s16_ tableNac[SOV_NUMS];
  _cap_s16_ tableNacUpdate[SOV_NUMS];
} ALIGNED_ATTRIBUTE CapacityDataType;

/**
 * @brief UpiInitCapacity
 *
 *  Initial capacity algorithm
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
extern void UpiInitCapacity(CapacityDataType *data);

/**
 * @brief UpiReadCapacity
 *
 *  Read capacity information
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
extern void UpiReadCapacity(CapacityDataType *data);

/**
 * @brief UpiTableCapacity
 *
 *  Look up capacity from table
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
extern void UpiTableCapacity(CapacityDataType *data);

/**
 * @brief UpiInitDsgCharge
 *
 *  Initialize data->dsgCharge value
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
extern void UpiInitDsgCharge(CapacityDataType *data);

/**
 * @brief UpiInitNacTable
 *
 *  Initialize NAC table
 *
 * @para  data  address of CapacityDataType
 * @return  _UPI_NULL_
 */
extern void UpiInitNacTable(CapacityDataType *data);

/**
 * @brief UpiSetChargerFull
 *
 *  Set charger full condition for capacity algorithm
 *
 * @para  data  address of CapacityDataType
 * @para  isFull  set _UPI_TRUE_ for full charge condition
 * @return  NULL
 */
extern void UpiSetChargerFull(CapacityDataType *data, _cap_bool_ isFull);

/**
 * @brief UpiAdjustCCRecord
 *
 *  Adjust CCRecord according to FCC 
 *
 * @para  data  address of CapacityDataType
 * @return  NULL
 */
extern void UpiAdjustCCRecord(CapacityDataType *data);

/**
 * @brief CalculateRsoc
 *
 *  RSOC = RM x 100 / FCC
 *
 * @para  rm  remaining capacity
 * @para  fcc full charged capacity
 * @return  relative state of charge
 */
extern _cap_u16_ CalculateRsoc(_cap_u32_ rm, _cap_u16_ fcc);

/**
 * @brief LkmCapacityInitial
 *
 *  Capacity initial procedure for linux kernel
 *
 * @para  data  address of CapacityDataType structure
 * @return  NULL
 */
extern void LkmCapacityInitial(CapacityDataType *data);

