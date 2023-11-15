# coding:utf_8
import json
import copy
import re
import sys
import os


class ZRBStmGenerator():

    def __init__(self, stm, dt_define, stm_name, path):
        self.__element_list = []
        self.__cond_list = []
        self.__rule_table = []
        self.__stm = stm
        self.__dt_define = dt_define
        self.__stm_name = stm_name
        self.__rule_names = []
        self.__headder = ''
        self.__definition = ''
        self.__rules = ''
        self.__constructors = ''
        self.__functions = ''
        self.__path = path

    def search_state(self, key, items):
        values = [x['id']
                  for x in items if 'state' in x and 'id' in x and x['state'] == key]
        return values[0] if values else None

    def search_fact_in_syntax(self, fact, syntax):
        # 現状FACT区切り文字をスペース" "、括弧"()"、イコール"="、アンド"&"、オア"|"にしている
        return re.search('[ ())=&|]+'+fact+'[ ())=&|]+|^'+fact+'[ ())=&|]+', syntax)

    def extract_fact_from_syntax(self, syntax):
        for fact in self.__dt_define:
            if self.search_fact_in_syntax(fact, syntax):
                return fact

    def get_stm_data(self, parent, id_str, element_list):
        child_id_num = 0
        for child in parent:
            child_id_num += 1
            child_id_str = id_str+'_' + str(child_id_num)
            if ('expression' in child):
                element_list.append(
                    dict(expression=child['expression'], id=child_id_str))
            if ('processing' in child):
                if (',' in child['processing']):
                    pro_list = child['processing'].split(",")
                    for proc in pro_list:
                        element_list.append(
                            dict(processing=proc, id=child_id_str))
                else:
                    element_list.append(
                        dict(processing=child['processing'], id=child_id_str))
            if ('transition' in child):
                element_list.append(
                    dict(transition=child['transition'], id=child_id_str))
            if ('condition' in child):
                self.get_stm_data(
                    copy.deepcopy(child['condition']), child_id_str, element_list)

    def make_rules(self):
        rule_num = 0
        for proc in filter(lambda e: e.get('processing'), self.__element_list):
            rule_num = rule_num+1
            proc_id = proc['id']
            proc_id_list = proc_id.split('_')
            filter_id = ''
            # 対象IDのprocessingがrule_tableに未登録の場合。同じIDのprocessingが複数あった場合に同じ内容のルールの重複登録を防ぐ
            if (not list(filter(lambda item: item['id'] == proc_id and 'processing' in item, self.__rule_table))):
                for id in proc_id_list:
                    filter_id = filter_id+id
                    rule_list = copy.deepcopy(
                        list(filter(lambda item: item['id'] == filter_id, self.__element_list)))
                    for rule in rule_list:
                        rule['rule'] = 'RULE'+str(rule_num)
                        if (rule['rule'] not in self.__rule_names):
                            self.__rule_names.append(rule['rule'])
                    self.__rule_table.extend(rule_list)
                    filter_id = filter_id+'_'

    def make_fact_for_states(self):
        self.__dt_define['FACT_RULE_STATE'] = {}
        states = filter(lambda e: e.get('state'), self.__rule_table)
        val = 0
        for row in states:
            key = 'FACT_RULE_'+row['id']
            if (key not in self.__dt_define['FACT_RULE_STATE']):
                self.__dt_define['FACT_RULE_STATE'][key] = val
                val = val+1

    def set_fact_to_cond_act(self):
        for key in self.__rule.keys():
            for name, syntax in self.__rule[key].items():
                cond_dict = {}
                cond_dict[key] = name
                cond_dict['syntax'] = syntax
                cond_dict['fact'] = []
                for fact in self.__dt_define:
                    if self.search_fact_in_syntax(fact, syntax):
                        cond_dict['fact'].append(fact)
                self.__cond_list.append(cond_dict)

    def make_rule_table(self):
        for state in filter(lambda e: e.get('state'), self.__rule_table):
            state['syntax'] = 'FACT_RULE_STATE == FACT_RULE_'+state['id']
            state['fact'] = 'FACT_RULE_STATE'

        for trans in filter(lambda e: e.get('transition'), self.__rule_table):
            target_state = self.search_state(
                trans['transition'], self.__element_list)
            trans['syntax'] = 'FACT_RULE_STATE = FACT_RULE_'+target_state
            trans['fact'] = 'FACT_RULE_STATE'

        multi_fact_list = []
        for cond in filter(lambda e: e.get('condition'), self.__cond_list):
            exp_list = filter(lambda e: e.get('expression'), self.__rule_table)
            for row in filter(lambda item: item['expression'] == cond['condition'], exp_list):
                # expressionのvalueとconditionのkeyを紐づけてruleファイルの判定式を格納
                row['syntax'] = cond['syntax']
                row['fact'] = cond['fact'][0]
                # 対応する判定式に紐づくFACTが複数ある場合はレコードをコピーしてFACTのみ異なる同じレコードを作成
                for fact in cond['fact'][1:]:
                    row2 = copy.deepcopy(row)
                    row2['fact'] = fact
                    multi_fact_list.append(row2)

        for cond in filter(lambda e: e.get('action'), self.__cond_list):
            exp_list = filter(lambda e: e.get('processing'), self.__rule_table)
            for row in filter(lambda item: item['processing'] == cond['action'], exp_list):
                # processingのvalueとactionのkeyを紐づけてruleファイルの判定式を格納
                row['syntax'] = cond['syntax']
                row['fact'] = cond['fact'][0]
                # 対応する判定式に紐づくFACTが複数ある場合はレコードをコピーしてFACTのみ異なる同じレコードを作成
                for fact in cond['fact'][1:]:
                    row2 = copy.deepcopy(row)
                    row2['fact'] = fact
                    multi_fact_list.append(row2)
        # 複数FACT用レコードをrule_tableに追加
        self.__rule_table.extend(multi_fact_list)

        for row in self.__rule_table:
            if (('expression' in row) or ('state' in row)):
                # if関数名登録
                row['if_func'] = 'int32_t Rule_if_'+row['id']+'()'
                if ('syntax' not in row):
                    # ruleファイルに条件・処理がなく、状態遷移モデルに条件・処理が直接書かれていた場合の処理
                    row['syntax'] = row['expression']
                    row['fact'] = self.extract_fact_from_syntax(row['syntax'])
            elif (('processing' in row) or ('transition' in row)):
                # then関数名登録
                row['then_func'] = 'void Rule_then_'+row['id']+'()'
                if ('syntax' not in row):
                    # ruleファイルに条件・処理がなく、状態遷移モデルに条件・処理が直接書かれていた場合の処理
                    row['syntax'] = row['processing']
                    row['fact'] = self.extract_fact_from_syntax(row['syntax'])
            else:
                pass

    def make_CPP_headder(self):
        self.__headder = ('# ifndef RBRULE_H\n'
                          '# define RBRULE_H\n'
                          '# include "' + 'RBLib.h"\n\n'
                          '/////////////////////\n'
                          '// Fact Definition\n'
                          '/////////////////////\n'
                          )
        for var_syntax, var_val in self.__fact.items():
            for val_syntax, val_val in var_val['values'].items():
                self.__headder = self.__headder + '#define ' + \
                    val_syntax+' '+str(val_val['value'])+'\n'

        self.__headder = self.__headder + '\nenum e_FACT {\n'
        key_list = list(self.__dt_define.keys())
        self.__headder = self.__headder + ' ' + key_list[0] + ' = 0, \n'
        for key in key_list[1:]:
            self.__headder = self.__headder + ' ' + key + ',\n'
        self.__headder = self.__headder + ' FACT_END\n};\n'

        self.__headder = self.__headder + ('\n/////////////////////\n'
                                           '// Rule Definition\n'
                                           '/////////////////////\n'
                                           )
        self.__headder = self.__headder + 'enum e_Rule {\n'

        rule_names = list(set([d.get('rule') for d in self.__rule_table]))
        rule_names.sort()
        self.__headder = self.__headder + ' ' + rule_names[0] + ' = 0, \n'
        for rule_name in rule_names[1:]:
            self.__headder = self.__headder + ' ' + rule_name + ',\n'
        self.__headder = self.__headder + ' RULE_END\n};\n'

        self.__headder = self.__headder + ('\n/////////////////////////\n'
                                           '// RuleQue Definition\n'
                                           '/////////////////////////\n'
                                           )
        for key in self.__dt_define.keys():
            self.__headder = self.__headder + '#define '+'RULE_QUE_MAX_' + key+' ' + \
                str(len(set([d.get('rule') for d in filter(lambda item: item['fact'] == key, self.__rule_table)])))+'\n'

        self.__headder = self.__headder + ('\n/////////////////////////\n'
                                           '// Function\n'
                                           '/////////////////////////\n'
                                           'extern void ZRB_RuleInitialize();\n'
                                           'extern fact_info_s* ZRB_GetFactPtr();\n'
                                           'extern rule_info_s* ZRB_GetRulePtr();\n'
                                           'extern void ZRB_SetPointer(rclcpp::Node* nodePointer);\n'
                                           '\n#endif\n')

    def make_CPP_code(self):
        self.__definition = ('#include <stdio.h>\n'
                             '#include <stdint.h>\n'
                             '#include <rclcpp/rclcpp.hpp>\n'
                             '#include "ZRB_Rule_' +
                             f'{self.__stm_name}.h'+'"\n\n'
                             '//////////////////////////\n'
                             '// Definition\n'
                             '//////////////////////////\n\n'
                             'rclcpp::Node* nPointer;\n'
                             'fact_info_s fact_info;\n'
                             'rule_info_s rule_info;\n'
                             'fact_s fact[FACT_END];\n'
                             'rule_s rule[RULE_END];\n'
                             )
        rule_func_list = []
        for rule_name in self.__rule_names:
            # ルール名でrule_tableをフィルタしてif_func,then_funcがあるレコードを重複を削除して検索し、rule_func_listに登録
            if_funcs = set([d.get('if_func') for d in filter(lambda item: item['rule'] == rule_name, self.__rule_table) if d.get('if_func') is not None])
            then_funcs = set([d.get('then_func') for d in filter(lambda item: item['rule'] == rule_name, self.__rule_table) if d.get('then_func') is not None])
            rule_func_list.append(dict(rule=rule_name, if_func=if_funcs, then_func=then_funcs))
            # if_func,then_funcがあるレコードのヒット数をカウントしてRule_if, Rule_thenテーブルの要素数として設定
            self.__definition = self.__definition + 'rule_if Rule_if_' + \
                rule_name + '[' + str(len(if_funcs))+'];\n'
            self.__definition = self.__definition + 'rule_then Rule_then_' + \
                rule_name + '[' + str(len(then_funcs))+'];\n'

        for key in self.__dt_define.keys():
            self.__definition = self.__definition + \
                'rule_que rule_que_' + key+'[RULE_QUE_MAX_'+key+'];\n'

        self.__rules = ('//////////////////////////\n'
                        '// Rules\n'
                        '//////////////////////////\n')

        if_func_fact_list = []
        if_func_list = set(
            [d.get('if_func') for d in self.__rule_table if d.get('if_func') is not None])
        for if_func in if_func_list:
            rows = list(filter(lambda item: item.get('if_func') == if_func, self.__rule_table))
            fact_name = list(set([d.get('fact') for d in rows]))
            syntax = list(set([d.get('syntax') for d in rows]))[0]
            if_func_fact_list.append(dict(if_func=if_func, fact=fact_name, syntax=syntax))

        for if_func in if_func_fact_list:
            self.__rules = self.__rules + \
                '\n'+if_func['if_func']+'\n{\n'
            self.__rules = self.__rules + \
                ' RCLCPP_INFO(nPointer->get_logger(),"' + \
                if_func['if_func'] + '\\n");\n'
            fact_num = ''
            for num, fact in enumerate(if_func['fact']):
                # 複数FACTの場合はfact_p1,2,3...と番号を振って変数を区別する。1つめのみ番号なし
                self.__rules = self.__rules + ' fact_s *fact_p' + \
                    fact_num + ' = &fact[' + fact + '];\n'
                if_func['syntax'] = re.sub('([ (|&!<>=])'+fact+'([ )|&!<>=])|^'+fact + '([ |&!<>=])|([ |&!<>=])' +
                                           fact+'$', '\\1\\4fact_p'+fact_num+'->fact_value\\2\\3', if_func['syntax'])
                fact_num = str(num+1)
            self.__rules = self.__rules + \
                ' if(' + if_func['syntax'].lstrip(' ') + ')\n'
            self.__rules = self.__rules + (
                ' {\n'
                '   RCLCPP_INFO(nPointer->get_logger(),"TRUE\\n");\n'
                '   return TRUE;\n'
                ' }\n'
                ' else\n'
                ' {\n'
                '   RCLCPP_INFO(nPointer->get_logger(),"FALSE\\n");\n'
                '   return FALSE;\n'
                ' }\n'
                '}\n'
            )

        then_func_fact_list = []
        then_func_list = set(
            [d.get('then_func') for d in self.__rule_table if d.get('then_func') is not None])
        for then_func in then_func_list:
            rows = list(filter(lambda item: item.get(
                'then_func') == then_func, self.__rule_table))
            fact_name = list(set([d.get('fact') for d in rows]))
            syntax = list(set([d.get('syntax') for d in rows]))
            then_func_fact_list.append(
                dict(then_func=then_func, fact=fact_name, syntax=syntax))
        for then_func in then_func_fact_list:
            self.__rules = self.__rules + \
                '\n'+then_func['then_func']+'\n{\n'
            self.__rules = self.__rules + \
                ' RCLCPP_INFO(nPointer->get_logger(),"' + \
                then_func['then_func'] + '\\n");\n'
            for fact in then_func['fact']:
                for syntax in then_func['syntax']:
                    if (self.search_fact_in_syntax(fact, syntax)):
                        syntax = re.sub('([ (|&!<>=])'+fact+'([ )|&!<>=])|^'+fact + '([ |&!<>=])|([ |&!<>=])' +
                                        fact+'$', '\\1\\4 fact[' + fact+'].fact_value\\2\\3', syntax)
                        self.__rules = self.__rules + \
                            syntax + ';\n'
                        break
            self.__rules = self.__rules + '}\n'

        self.__constructors = self.__constructors + ('\nvoid construct_rule()\n'
                                                     '{\n'
                                                     ' rule_if *rule_if_p;\n'
                                                     ' rule_then *rule_then_p;\n'
                                                     ' rule_s *rule_p;\n')
        for row in rule_func_list:
            rule_name = row.get('rule')
            self.__constructors = self.__constructors + \
                '\n rule_if_p = Rule_if_' + rule_name + ';\n'
            if_cnt = 0
            for if_cnt, if_func in enumerate(row.get('if_func')):
                self.__constructors = self.__constructors + (' rule_if_p['+str(if_cnt)+'].IF_ptr = &'+if_func.replace('int32_t ', '').replace('(', '').replace(')', '')+';\n'
                                                             ' rule_if_p['+str(if_cnt)+'].result = FALSE;\n')
                rule_table_row = list(filter(lambda item: item.get(
                    'rule') == rule_name and item.get('if_func') == if_func, self.__rule_table))
                rule_table_row[0]['if_index'] = str(if_cnt)
            self.__constructors = self.__constructors + \
                ' rule_then_p = Rule_then_' + rule_name + ';\n'
            then_cnt = 0
            for then_cnt, then_func in enumerate(row.get('then_func')):
                self.__constructors = self.__constructors + \
                    ' rule_then_p['+str(then_cnt)+'].THEN_ptr = &' + \
                    then_func.replace('void ', '').replace(
                        '(', '').replace(')', '')+';\n'
            self.__constructors = self.__constructors + (' rule_p = &rule['+rule_name+'];\n'
                                                         ' rule_p->rule_ID = '+rule_name+';\n')
            self.__constructors = self.__constructors + (' rule_p->rule_IF_max = '+str(if_cnt+1) + ';\n'
                                                         ' rule_p->rule_IF_first_ptr = rule_if_p;\n'
                                                         ' rule_p->rule_THEN_max = ' +
                                                         str(then_cnt+1) +
                                                         ';\n'
                                                         ' rule_p->rule_THEN_first_ptr = rule_then_p;\n')
            self.__constructors = self.__constructors + (' rule_p->agenda_previous_ptr = NULL;\n'
                                                         ' rule_p->agenda_next_ptr = NULL;\n'
                                                         ' rule_p->fired_previous_ptr = NULL;\n'
                                                         ' rule_p->fired_next_ptr = NULL;\n')
        self.__constructors = self.__constructors + '}\n'

        self.__constructors = self.__constructors + ('\nvoid construct_ruleQue()\n'
                                                     '{\n'
                                                     ' rule_que *rule_que_p;\n\n')
        for key in self.__dt_define.keys():
            # rule_tableを対象FACTでif関数があるレコードに絞り込む
            fact_rows = list(filter(
                lambda item: item['fact'] == key and 'if_func' in item.keys(), self.__rule_table))
            if (fact_rows != []):
                # if関数のあるFACTのみrule_queに出力する
                self.__constructors = (self.__constructors + ' //'+key+'\n'
                                       ' rule_que_p = rule_que_' + key+';\n')
                for cnt, row in enumerate(fact_rows):
                    rq_head = ' rule_que_p[' + str(cnt) + '].'
                    self.__constructors = self.__constructors + \
                        rq_head+'rule_ID_no = '+row['rule']+';\n'
                    self.__constructors = self.__constructors + \
                        (rq_head+'rule_IF_no = '+row['if_index']+';\n'
                         + rq_head+'rule_group_no = 0;\n'
                         + rq_head+'rule_priority = 0;\n')
                    if (cnt == len(fact_rows)-1):
                        self.__constructors = self.__constructors + rq_head+'RQ_next_ptr = NULL;\n'
                    else:
                        self.__constructors = self.__constructors + rq_head + \
                            'RQ_next_ptr = &rule_que_p['+str(cnt+1)+'];\n'
                    if (cnt == 0):
                        self.__constructors = self.__constructors + rq_head+'RQ_previous_ptr = NULL;\n'
                    else:
                        self.__constructors = self.__constructors + rq_head + \
                            'RQ_previous_ptr = &rule_que_p['+str(cnt-1)+'];\n'
        self.__constructors = self.__constructors + '}\n'
        self.__constructors = self.__constructors + ('void initial_fact_common_info(fact_s *fact_p)\n'
                                                     '{\n'
                                                     ' fact_p->Rule_Que_current_no = 0;\n'
                                                     ' fact_p->fact_status = READY;\n'
                                                     ' fact_p->fact_value = -1;\n'
                                                     ' fact_p->fact_priority = 0;\n'
                                                     ' fact_p->interval_timer = 0;\n'
                                                     ' fact_p->fact_History_max = 0;\n'
                                                     ' fact_p->fact_History_current_no = 0;\n'
                                                     ' fact_p->fact_History_first_ptr = NULL;\n'
                                                     ' fact_p->time_support = 0;\n'
                                                     ' fact_p->fact_time = 0;\n'
                                                     ' fact_p->fact_time_History_first_ptr = NULL;\n'
                                                     '}\n')

        self.__constructors = self.__constructors + ('\nvoid construct_fact()\n'
                                                     '{\n'
                                                     ' fact_s *fact_p;\n')
        for key in self.__dt_define.keys():
            self.__constructors = self.__constructors + \
                (' fact_p = &fact['+key+'];\n'
                 + ' fact_p->fact_ID = '+key+';\n'
                 + ' fact_p->Rule_Que_Max = RULE_QUE_MAX_'+key+';\n'
                 + ' fact_p->Rule_Que_first_ptr = rule_que_'+key+';\n'
                 + ' fact_p->Rule_Que_last_ptr = &rule_que_' +
                 key+'[RULE_QUE_MAX_'+key+'- 1];\n'
                 + ' initial_fact_common_info(fact_p);\n')
        self.__constructors = self.__constructors + '}\n'

        self.__functions = self.__functions + ('\nvoid ZRB_RuleInitialize()\n{\n'
                                               ' RCLCPP_INFO(nPointer->get_logger(),"ZRB_RuleInitialize()\\n");\n'
                                               ' construct_rule();\n'
                                               ' construct_fact();\n'
                                               ' construct_ruleQue();\n'
                                               '}\n')
        self.__functions = self.__functions + ('\nfact_info_s* ZRB_GetFactPtr()\n{\n'
                                               ' RCLCPP_INFO(nPointer->get_logger(),"ZRB_GetFactPtr() return[%p]\\n", &fact[0]);\n'
                                               ' fact_info.fact_max = FACT_END;\n'
                                               ' fact_info.fact_ptr = fact;\n'
                                               ' return &fact_info;\n'
                                               '}\n')
        self.__functions = self.__functions + ('\nrule_info_s* ZRB_GetRulePtr()\n{\n'
                                               ' RCLCPP_INFO(nPointer->get_logger(),"ZRB_GetRulePtr() return[%p]\\n", &rule[0]);\n'
                                               ' rule_info.rule_group_max = 0;\n'
                                               ' rule_info.rule_group_p = 0;\n'
                                               ' rule_info.rule_max = RULE_END;\n'
                                               ' rule_info.rule_ptr = rule;\n'
                                               ' return &rule_info;\n'
                                               '}\n')
        self.__functions = self.__functions + ('\nvoid ZRB_SetPointer(rclcpp::Node* nodePointer)\n{\n'
                                               ' nPointer = nodePointer;\n'
                                               '}\n')

    def generation(self):
        for id_num, action in enumerate(self.__stm['stm']['action']):
            state = action['state']
            id_str = 'S'+str(id_num+1)
            self.__element_list.append(dict(state=state, id=id_str))
            self.get_stm_data(action['condition'], id_str, self.__element_list)
        self.make_rules()
        self.make_fact_for_states()
        self.set_fact_to_cond_act()
        self.make_rule_table()
        self.make_CPP_headder()
        print("<< element_list >>")
        # print(self.__element_list)
        self.make_CPP_code()
        print("<< rule_table >>")
        # print(self.__rule_table)
        cpp_str = self.__definition + self.__rules + \
            self.__constructors + self.__functions

        headder = open(os.path.join(self.__path, f'ZRB_Rule_{self.__stm_name}.h'), 'w+', encoding='UTF-8')
        headder.write(self.__headder)
        headder.close()
        cpp = open(os.path.join(self.__path, f'ZRB_Rule_{self.__stm_name}.cpp'), 'w+', encoding='UTF-8')
        cpp.write(cpp_str)
        cpp.close()
