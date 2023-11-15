/********** sys_def.h **********/
/* CONFIDENTIAL                */
/* Copyright CATS 2016         */
/*******************************/
#pragma once

// マクロ定義

#ifndef NULL
#define NULL 0
#endif // !NULL

#define TRUE  1
#define FALSE 0

#define OK 1
#define NG 0

/* Rule Que order */
#define FIX 1		//固定
#define VARIOUS 0	//可変

typedef unsigned int uint;


#define HISTORY_INIT	-1		// factのValueで使わない値を入れる

// RuleGroup未所属
#define RULE_GROUP_NON	0xFFFFFFFF

// 戻り値定義
#define RET_OK					1
#define RET_NG					2
#define RET_RULE_PERFORMED		3
#define RET_RULE_NOT_PERFORMED	4
#define RET_ERR_NO_QUE			5
#define RET_ERR_NULL			6
#define RET_NO_DATA				7
#define RET_OVER				8

// 列挙定義
// fact状態
enum e_factStatus
{
	INIT = 1,
	READY,
	WAIT,
	ACTIVE
};

// WMキュー方式
enum e_WMQueStyle
{
	FIFO = 1,
	LIFO,
	PRIO
};

// 条件関数ポインタ
typedef int(*IF_funcptr)();
// アクション関数ポインタ
typedef void(*THEN_funcptr)();

typedef struct rule_if
{
	IF_funcptr IF_ptr;
	uint result;		//式評価結果 OK / NG
}rule_if;

typedef struct rule_then
{
	THEN_funcptr THEN_ptr;
}rule_then;

// ルールキュー構造
typedef struct rule_que
{
	uint rule_ID_no;
	uint rule_IF_no;
	uint rule_group_no; //rule_group_no == 0はグループ無、グループ有の場合は1から開始
	uint rule_priority;	//rule_priority==0が最強、 0xFFFF FFFF(4,294,967,295)が最弱
	struct rule_que *RQ_next_ptr;
	struct rule_que *RQ_previous_ptr;
}rule_que;

// rule構造
typedef struct rule_s
{
	uint rule_ID;
	uint rule_group_ID;
	uint rule_priority;	//rule_priority==0が最強、 0xFFFF FFFF(4,294,967,295)が最弱
	uint rule_IF_max;
	struct rule_if *rule_IF_first_ptr;
	struct rule_if *rule_IF_last_ptr;
	uint rule_group_max;
	uint * rule_group_first_ptr;
	uint rule_THEN_max;
	uint rule_THEN_current_no;
	struct rule_then *rule_THEN_first_ptr;
	struct rule_s *agenda_previous_ptr;
	struct rule_s *agenda_next_ptr;
	struct rule_s *fired_previous_ptr;
	struct rule_s *fired_next_ptr;
}rule_s;

// rule_group内のrule
typedef struct rule_group_rules_s
{
	rule_s *rule_p;
}rule_group_rules_s;

// rule_group構造
typedef struct rule_group_s
{
	uint rules_max;
	rule_group_rules_s *rules_ptr;
	rule_s *fired_rule_ptr;
}rule_group_s;

// rule全体構造
typedef struct rule_info
{
	uint rule_group_max;
	rule_group_s *rule_group_p;
	uint rule_max;
	rule_s *rule_ptr;
}rule_info_s;

// fact構造
typedef struct fact_s
{
	uint fact_ID;						// ID 配列の要素Indexと同義
	int fact_value;						// Factの最新値
	uint fact_status;					// Factの状態。WMから削除するとWAITになり評価対象外となる
	uint fact_priority;					// fact_priority==0が最強、 0xFFFF FFFF(4,294,967,295)が最弱
	uint interval_timer;				// 単位をコンストラクタで設定する 例えば 24(ms) または 2(tick)
	int fact_History_max;				// Factの履歴最大数
	int fact_History_current_no;		// Factの現在のIndex
	int fact_History_count;				// Factの現在の履歴数
	int *fact_History_first_ptr;		// Factの値を履歴で管理する配列の先頭ポインタ
	int fact_History_full_flag;			//リングバッファが初期状態から充填された(TRUE)か否か(FALSE)
	int time_support;					// 時間サポートの有無。TRUEかFALSE
	int fact_time;						// Factの最新値の登録時間
	int *fact_time_History_first_ptr;	// Factの登録時間を履歴で管理する配列の先頭ポインタ
	uint Rule_Que_Max;					// Factが関連するRuleのIFを管理するチェーンの数
	uint Rule_Que_current_no;			// 未使用
	struct rule_que *Rule_Que_first_ptr;// RuleQueの先頭ポインタ
	struct rule_que *Rule_Que_last_ptr;	// RuleQueの最終ポインタ
	struct fact_s *WM_next_ptr;			// WMのチェーン用の次のFactポインタ
	struct fact_s *WM_previous_ptr;		// WMのチェーン用の前のFactポインタ
	uint user_wk1;						// ユーザーが自由に使える領域
	uint user_wk2;						// ユーザーが自由に使える領域
}fact_s;

// fact情報
typedef struct fact_info
{
	uint fact_max;
	fact_s *fact_ptr;
}fact_info_s;

// ルールエンジン制御構造体
typedef struct wmb
{
	uint WM_Que_num;					// 登録されているFactの数
	fact_s *WM_Que_first_ptr;			// 登録されているFactの先頭ポインタ
	fact_s *WM_Que_last_ptr;			// 登録されているFactの最終ポインタ
	rule_s *PM_agenda_first_ptr;		// 発火可能Ruleの先頭ポインタ
	rule_s *PM_agenda_last_ptr;			// 発火可能Ruleの最終ポインタ
	rule_s *PM_fired_first_ptr;			// 発火済Ruleの先頭ポインタ
	rule_s *PM_fired_last_ptr;			// 発火済Ruleの最終ポインタ
	uint PM_rule_group_max;				// RuleGroupの数
	rule_group_s *PM_rule_group_ptr;	// RuleGroupの先頭ポインタ
	uint PM_rule_max;					// Ruleの数
	rule_s *PM_rule_ptr;				// Ruleの先頭ポインタ
	uint WM_fact_max;					// Factの数
	fact_s *WM_fact_ptr;				// Factの先頭ポインタ
	int base_time;						// 基準時間
}wmb_s;

typedef struct before_value_s {
	int err_code;
	int fact_value;
	int before_time;
}before_value_s;

// 関数宣言
// 変数をWMにセットする
int SetWMQue(struct fact_s *fact_p, int order);
int SetWMQueFIFO(struct fact_s *fact_p);
int SetWMQueLIFO(struct fact_s *fact_p);
int SetWMQuePRIO(struct fact_s *fact_p);

int ChangeWMQuePriority(struct fact_s *fact_p, uint priority);
void WMQ_PrioritySort(struct fact_s *fact_p);

// 歴史情報処理
void SetHistory(struct fact_s *fact_p, int value);

// ルールをキューにセットする
int SetRuleQue(struct fact_s *fact_p, struct rule_que * rq_p, int order);
int SetRuleQueFIFO(struct fact_s *fact_p, struct rule_que * rq_p);
int SetRuleQueLIFO(struct fact_s *fact_p, struct rule_que * rq_p);
int SetRuleQuePRIO(struct fact_s *fact_p, struct rule_que * rq_p);
int ChangeRuleQuePRIO(struct fact_s *fact_p, struct rule_que * rq_p, uint priority);
void RQ_PrioritySort(struct fact_s *fact_p, struct rule_que * rq_p);

int DeleteRuleQue(struct fact_s *fact_p, struct rule_que * rq_p);

// 評価実施
void RuleExec();

// ルールキュー検索実施
int RuleCall(struct fact_s *fact_p, uint group_no);
int RuleCall_No_Group(struct fact_s *fact_p);
int RuleCall_Group(struct fact_s *fact_p, uint group_no);

// ルール条件実行
int RuleIFExec(struct fact_s *fact_p, uint rule_ID_no, uint rule_IF_no);

// ルールアクション実行
void rule_then_Exec(uint rule_ID_no);

// rule_if初期化
void Init_rule_if_result(struct rule_if *rule_IF_ptr, uint rule_if_max);

// ワーキングメモリクリア
void WMClear();

// insert
int WM_insert(struct fact_s *fact_p, int value, int order);

// 指定factのみ値登録と再評価
int WM_single_insert(struct fact_s *fact_p, int value, int order, uint group_no);

// make 
int WM_make(struct fact_s *fact_p, int value, int order);

// タイムスタンプを変わらず、値を変えて、再評価する
// set_modify 
int WM_set_modify(struct fact_s *fact_p, int value, int order);

// タイムスタンプを変わらず、値を変えず、再評価する
// modify */
int WM_modify(struct fact_s *fact_p, int order);

// タイムスタンプを変わらず、値を変えず、指定したfactだけを再評価する
// ワーキングメモリ登録されているかは無視する
int WM_single_modify(struct fact_s *fact_p, uint group_no);

// 過去の値を取り出す
// get_before 
int WM_get_before(struct fact_s *fact_p, int before_time);

// 指定tick以前の過去の最新値を取り出す
// get_before_tick
int WM_get_before_time(struct fact_s *fact_p, struct before_value_s *before_value_p);

// 過去の値を取り出す
// get_before
int WM_get_before_index(struct fact_s *fact_p, int before_time, int index);

// 指定tick以前の過去の最新値を取り出す
// get_before_tick
int WM_get_before_time_index(struct fact_s *fact_p, struct before_value_s *before_value_p, int index);

// ワーキングメモリから削除する
// delete 
int DeleteWMQue(struct fact_s *fact_p);


// Rulex_if[x].resultをクリアする
void Clear_if_result(struct rule_s *rule_p);

// fact_x_HistoryQue[x]をクリアする
void Clear_History(struct fact_s *fact_p);

//factのルールキュー関係をクリアする
void Clear_fact_RuleQue(struct fact_s *fact_p);

//rule_que(fact_x_RuleQ[])のルールキュー関係をクリアする
void Clear_RuleQue(struct rule_que *rq_p);

// 対象RuleのIFが全てTRUEか判定(応答はTRUE/FALSE)
int WM_CheckRuleIfResult(rule_s *rule_p);

// 対象Ruleの全IFがTRUEでagenda/fired未登録ならagendaに登録する
int WM_AddAgendaRule(struct rule_s *rule_p);

// agendaからRuleを削除
void WM_DelAgendaRule(rule_s *rule_p);

// firedにRuleを登録
int WM_AddFiredRule(rule_s *rule_p);

// firedからRuleを削除
void WM_DelFiredRule(rule_s *rule_p);

// agendaからRuleGroup内のRuleを削除
void WM_DelAgendaRuleGroup(uint rule_group_ID);

// Ruleを実行する
void WM_Action(void);

// agendaに登録済のRuleを順次実行する
void WM_ExecAgendaRule();

// firedに登録済のRuleを初期化する(削除)
void WM_ResetFiredRule();

// ルールエンジンの制御領域を初期化する
void ZRB_Initialize(fact_info_s *fact_info_p, rule_info_s *rule_info_p);

// Factを登録し、Ruleを評価する
int ZRB_AddFact(uint fact_index, int value);

// Factに関連するRuleを評価する
int ZRB_CheckFact(fact_s *fact_p);

// Ruleを実行する
void ZRB_Action(void);

// WMに登録したFactに関連するRuleのIF結果をリセットする
int ZRB_ResetRuleIF();

// 基準時間を設定する
void ZRB_SetTime(int time);

// 基準時間を取得する
int ZRB_GetTime();

// 指定個数前の値を取り出す
int ZRB_GetFactBefore(fact_s *fact_p, int before_count);

// 指定時間前の値を取り出す
int ZRB_GetFactBeforeTime(fact_s *fact_p, before_value_s *before_value_p);

// agendaの一覧を出力する
void WM_DebugAgendaPrint();

// firedの一覧を出力する
void WM_DebugFiredPrint();
