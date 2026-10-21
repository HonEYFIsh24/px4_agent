#!/usr/bin/env python3
"""One actual production single-VLM telemetry turn, with a read-only transport.

Default prints a plan. --execute runs one production Agent graph invocation;
provider retries are disabled by the unchanged single-VLM implementation.
The real bridge is checked grounded before and after. Services are never started
and model/tool failures are preserved; no flight or safety commands are sent.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sys
import time
from urllib.parse import urlsplit
import uuid

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from client import BridgeClient

DEFAULT_ENV=Path('/home/dut/ros_agent/mcp-client-langchain/.env')
PROMPT=('本次只读遥测检查，不执行任何飞行、停止、降落、相机或参数操作。'
        '必须且只调用一次 get_drone_state，传入 view="all"，不要其他字段。'
        '根据这一次真实工具结果，最终只返回一个JSON对象：'
        '{"armed":布尔值,"landed_state":整数,"vio_ready":vio.ready的布尔值,'
        '"used_for_flight_control":external_vision.used_for_flight_control的布尔值或缺失时null}。'
        '注意独立 vio.used_for_flight_control 与 external_vision.used_for_flight_control 是不同字段。'
        '不得根据过去状态或模型记忆填造结果。工具报错则如实报告失败，不重试。')


def require_ground(state):
    if (not isinstance(state,dict) or state.get('simulation_only') is not True
            or state.get('armed') is not False or state.get('landed_state')!=1
            or (state.get('mission') or {}).get('status')=='running'):
        raise RuntimeError('Read-only production diagnostic requires grounded, disarmed local simulation with no running mission')


def sanitize(value,secrets=()):
    from scripts.diagnose_model_connection import clean_text
    if isinstance(value,str):return clean_text(value,secrets,limit=120000)
    if isinstance(value,dict):return {k:sanitize(v,secrets) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [sanitize(v,secrets) for v in value]
    if isinstance(value,(int,float,bool)) or value is None:return value
    return clean_text(str(value),secrets,limit=120000)


class ReadonlyBridge(BridgeClient):
    supports_stereo_localization=False
    agent_architecture='single_vlm'
    def __init__(self,url='http://127.0.0.1:8765'):
        parsed=urlsplit(url)
        if parsed.scheme!='http' or parsed.hostname not in ('localhost','127.0.0.1','::1'):
            raise ValueError('Production read-only diagnostic requires the local bridge endpoint')
        super().__init__(url);self.transport_events=[];self.audit=[];self.blocked=[]

    def reject(self,operation,arguments=None):
        event={'operation':operation,'arguments':arguments,'blocked_before_transport':True,'wall_s':time.time()}
        self.blocked.append(event)
        raise RuntimeError('Read-only telemetry diagnostic blocked '+operation+' before bridge/ROS transport')

    def request(self,path,payload=None):
        if payload is not None:self.reject('POST '+path,copy.deepcopy(payload))
        if path!='/state':self.reject('GET '+path)
        started=time.time()
        result=super().request(path)
        self.transport_events.append({'method':'GET','path':path,'started_at_wall_s':started,
                                      'completed_at_wall_s':time.time(),'result':copy.deepcopy(result)})
        return result

    def execute(self,steps,description='',on_progress=None):self.reject('execute_mission',{'steps':steps,'description':description})
    def wait_for(self,accepted,on_progress=None):self.reject('wait_for_flight',accepted)
    def capture_camera(self,side='left'):self.reject('capture_camera',{'side':side})
    def capture_stereo(self):self.reject('capture_stereo')
    def localize_stereo(self,*args,**kwargs):self.reject('localize_stereo')
    def record(self,name,result):
        self.audit.append({'tool':name,'result':copy.deepcopy(result),'wall_s':time.time()})
        return result


def message_record(message):
    return {'type':getattr(message,'type',None),'name':getattr(message,'name',None),
            'id':getattr(message,'id',None),'content':getattr(message,'content',None),
            'tool_calls':getattr(message,'tool_calls',None),
            'tool_call_id':getattr(message,'tool_call_id',None),
            'status':getattr(message,'status',None),
            'usage_metadata':getattr(message,'usage_metadata',None),
            'response_metadata':getattr(message,'response_metadata',None)}


def validate_messages(messages,client):
    tools=[m for m in messages if m.get('type')=='tool']
    calls=[call for m in messages if m.get('type')=='ai' for call in m.get('tool_calls') or []]
    telemetry=[m for m in tools if m.get('name')=='get_drone_state']
    final=next((m.get('content') for m in reversed(messages) if m.get('type')=='ai' and not m.get('tool_calls')),None)
    checks=[]
    def check(name,value,observed):checks.append({'name':name,'passed':bool(value),'observed':observed})
    check('one_production_telemetry_tool_call',len(calls)==1 and calls[0].get('name')=='get_drone_state'
          and calls[0].get('args')=={'view':'all'},calls)
    check('one_real_tool_result',len(tools)==1 and len(telemetry)==1,len(tools))
    check('no_blocked_mutation_or_unrequested_observation',not client.blocked,client.blocked)
    parsed=observed=None
    try:
        observed=json.loads(telemetry[0]['content']) if len(telemetry)==1 else None
        raw=final.strip() if isinstance(final,str) else ''
        if raw.startswith('```'):
            raw=raw.split('\n',1)[1].rsplit('```',1)[0].strip()
        parsed=json.loads(raw)
        if not isinstance(parsed,dict):raise ValueError('Final JSON must be an object')
        expected={'armed':observed['armed'],'landed_state':observed['landed_state'],
                  'vio_ready':observed['vio']['ready'],
                  'used_for_flight_control':(observed.get('external_vision') or {}).get('used_for_flight_control')}
        match=(set(parsed)==set(expected) and type(parsed.get('armed')) is bool
            and type(parsed.get('landed_state')) is int and type(parsed.get('vio_ready')) is bool
            and (parsed.get('used_for_flight_control') is None or type(parsed.get('used_for_flight_control')) is bool)
            and parsed==expected)
        check('final_reply_matches_actual_tool_telemetry',match,{'actual_reply':parsed,'expected':expected})
    except (ValueError,KeyError,TypeError,IndexError) as exc:
        check('final_reply_matches_actual_tool_telemetry',False,str(exc))
    return {'passed':all(c['passed'] for c in checks),'checks':checks,'final_reply':final,'parsed_reply':parsed}


def run(args):
    from agent import build_agent,flight_tools
    from agent_config import runtime_configuration
    from langchain_core.callbacks import BaseCallbackHandler
    from langchain_core.messages import HumanMessage
    from langchain_core.utils.function_calling import convert_to_openai_tool
    from scripts.diagnose_model_connection import exception_record,safe_uri
    args.output.mkdir(parents=True,exist_ok=False)
    report={'schema_version':1,'source':'production_single_vlm_readonly_telemetry','started_at_wall_s':time.time(),
            'actual_agent_graph_invocations':0,'actual_flight_commands_sent':0,'automatic_model_retries':0,
            'passed':False,'prompt':PROMPT,'thread_id':'production-telemetry-'+uuid.uuid4().hex,
            'recursion_limit':6,'max_concurrency':1}
    secrets=[];client=None;callbacks=[];messages=[];graph=None;invocation=None
    try:
        architecture,model_config,vision_config=runtime_configuration(args.env_file)
        secrets=[model_config.get('api_key'),(vision_config or {}).get('api_key')]
        if architecture!='single_vlm':raise RuntimeError('Configured production architecture is not single_vlm; do not change it for this test')
        report['model']={'architecture':architecture,'name':model_config['model'],'endpoint':safe_uri(model_config.get('base_url')),
                         'api_key_configured':bool(model_config.get('api_key'))}
        client=ReadonlyBridge(args.bridge_url)
        initial=client.state();require_ground(initial);report['initial_state']=initial
        schemas=[convert_to_openai_tool(tool) for tool in flight_tools(client)]
        report['production_flight_tool_schemas']=schemas
        class Callback(BaseCallbackHandler):
            def on_llm_end(self,response,**kwargs):
                callbacks.append({'event':'model_output','wall_s':time.time(),
                    'messages':[message_record(generation.message) for row in response.generations for generation in row
                                if hasattr(generation,'message')]})
            def on_llm_error(self,error,**kwargs):
                callbacks.append({'event':'model_error','wall_s':time.time(),'error':exception_record(error,secrets)})
            def on_tool_start(self,serialized,input_str,**kwargs):
                callbacks.append({'event':'tool_start','wall_s':time.time(),'tool':serialized.get('name'),
                                  'input':input_str,'inputs':kwargs.get('inputs')})
            def on_tool_end(self,output,**kwargs):
                callbacks.append({'event':'tool_end','wall_s':time.time(),
                                  'output':message_record(output) if hasattr(output,'content') else output})
            def on_tool_error(self,error,**kwargs):
                callbacks.append({'event':'tool_error','wall_s':time.time(),'error':exception_record(error,secrets)})
        graph=build_agent(client,model_config,vision_config=vision_config,architecture=architecture)
        invocation={'configurable':{'thread_id':report['thread_id']},'recursion_limit':6,
                    'max_concurrency':1,'callbacks':[Callback()]}
        report['actual_agent_graph_invocations']=1
        result=graph.invoke({'messages':[HumanMessage(content=PROMPT,id='telemetry-turn-'+uuid.uuid4().hex)]},invocation)
        messages=[message_record(m) for m in result['messages']]
        report['validation']=validate_messages(messages,client)
        report['passed']=report['validation']['passed']
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=exception_record(exc,secrets)
        if graph is not None and invocation is not None:
            try:messages=[message_record(m) for m in graph.get_state(invocation).values.get('messages',[])]
            except Exception as state_error:report['checkpoint_error']=exception_record(state_error,secrets)
        print('PRODUCTION_TELEMETRY_FAILURE '+json.dumps(sanitize(report['error'],secrets),ensure_ascii=False),flush=True)
    finally:
        if client is not None:
            try:
                final=client.state();require_ground(final);report['final_state']=final;report['final_ground_disarmed']=True
            except Exception as exc:
                report['final_state_error']=exception_record(exc,secrets);report['final_ground_disarmed']=False;report['passed']=False
            report.update(transport_events=client.transport_events,blocked_attempts=client.blocked,model_traces=client.audit)
        report.update(callback_events=callbacks,messages=messages,finished_at_wall_s=time.time())
        report['duration_wall_s']=report['finished_at_wall_s']-report['started_at_wall_s']
        report['provider_request_count']=sum(event['event'] in ('model_output','model_error') for event in callbacks)
        if report['actual_agent_graph_invocations']!=1:report['passed']=False
        (args.output/'validation.json').write_text(json.dumps(sanitize(report,secrets),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/vio_fusion/model_diagnostics/production_telemetry_01')
    parser.add_argument('--env-file',type=Path,default=DEFAULT_ENV)
    parser.add_argument('--bridge-url',default='http://127.0.0.1:8765')
    args=parser.parse_args(argv)
    if not args.execute:
        print(json.dumps({'execute':False,'actual_provider_calls':False,'actual_flight_commands_sent':False,
            'production_agent':'agent.build_agent/single_vlm','prompt':PROMPT,'recursion_limit':6,'max_concurrency':1,
            'actual_agent_invocations_if_executed':1,'automatic_retries':0},ensure_ascii=False,indent=2))
        return 0
    value=run(args)
    print(json.dumps({'report':str(args.output/'validation.json'),'passed':value['passed'],
                     'provider_request_count':value['provider_request_count'],'actual_flight_commands_sent':0},ensure_ascii=False))
    return 0 if value['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
