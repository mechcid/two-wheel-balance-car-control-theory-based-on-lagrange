"""With an isolated Gazebo launch already running, invoke real Tk button callbacks.
Uses current ROS_DOMAIN_ID/GZ_PARTITION; no K or model parameters are changed.
"""
import json,time,sys
from pathlib import Path
import tkinter as tk
import rclpy
from PIL import ImageGrab
from balance_sim.interaction import Interaction
from balance_sim.panel import Panel

out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=True)
rclpy.init(args=['--ros-args','-r','__node:=balance_interaction_test'])
n=Interaction();root=tk.Tk();p=Panel(root,n)
state={'stage':'wait','deadline':time.monotonic()+150,'events':[],'peaks':[0.,0.],'index':0}
actions=['forward','backward','yaw_left','yaw_right']
def finish(error=None):
    state['error']=error
    (out/'buttons.json').write_text(json.dumps(state,ensure_ascii=False,indent=2))
    p.stop()
def tick():
    try:
        if time.monotonic()>state['deadline']:raise RuntimeError('UI test timeout')
        stage=state['stage']
        if stage=='wait' and n.running and n.state and abs(n.state[2])<.003:
            p.buttons['pause'].invoke();state['stage']='pausing'
        elif stage=='pausing' and n.pending is None and not n.running:
            state['paused_at']=n.sim_time;state['wall']=time.monotonic();state['stage']='paused'
        elif stage=='paused' and time.monotonic()-state['wall']>1:
            assert abs(n.sim_time-state['paused_at'])<.002
            state['events'].append({'button':'pause','clock_stopped':True})
            p.buttons['resume'].invoke();state['stage']='resuming'
        elif stage=='resuming' and n.running and n.pending is None:
            state['events'].append({'button':'resume','clock_running':True});state['stage']='pulse'
        elif stage=='pulse':
            action=actions[state['index']]
            p.buttons[action].invoke();assert n.until is not None,n.status
            state['at']=n.sim_time;state['peaks']=[0.,0.];state['stage']='response'
        elif stage=='response':
            for i,j in enumerate([2,1]):state['peaks'][i]=max(state['peaks'][i],abs(n.state[j]))
            if n.sim_time-state['at']>6:
                action=actions[state['index']]
                assert n.until is None
                assert abs(n.state[2])<.003 and abs(n.state[1])<.005, n.state
                assert state['peaks'][0 if state['index']<2 else 1]>1e-4
                state['events'].append({'button':action,'peak_pitch':state['peaks'][0],'peak_yaw':state['peaks'][1],'final_state':n.state,'force_cleared':True})
                state['index']+=1
                state['stage']='pulse' if state['index']<len(actions) else 'clear_test'
        elif stage=='clear_test':
            p.buttons['forward'].invoke();assert n.until is not None
            p.buttons['clear'].invoke();assert n.until is None
            state['events'].append({'button':'clear','cleared':True})
            root.lift();state['stage']='screenshot';state['wall']=time.monotonic()
        elif stage=='screenshot' and time.monotonic()-state['wall']>.5:
            x,y=root.winfo_rootx(),root.winfo_rooty()
            ImageGrab.grab(bbox=(x,y,x+root.winfo_width(),y+root.winfo_height())).save(out/'panel.png')
            state['stage']='passed';finish();return
        root.after(20,tick)
    except Exception as exc:finish(str(exc))
root.after(100,tick)
try:root.mainloop()
finally:
    n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
print(json.dumps(state,ensure_ascii=False,indent=2))
if state.get('error'):sys.exit(1)
