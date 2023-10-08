#ifndef _FDB_CFG_H_
#define _FDB_CFG_H_

/* using KVDB feature */
#define FDB_USING_KVDB

/* using TSDB (Time series database) feature */
#define FDB_USING_TSDB

/* Using FAL storage mode */
#define FDB_USING_FAL_MODE

#ifdef FDB_USING_FAL_MODE
#define FDB_WRITE_GRAN 32
#endif

/* log print macro. default EF_PRINT macro is printf() */
#define FDB_PRINT(...)

/* print debug information */
#define FDB_DEBUG_ENABLE

#endif /* _FDB_CFG_H_ */
