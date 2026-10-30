"""Read HDMI experiment state or request one labelled on-board capture.

No motor commands. Command ACK is followed by capture-generation polling.
"""
import argparse,json,time
from pathlib import Path
from zlink_board_console import Console


def checked(console,kind,axis=0,payload=None):
    r=console.command(kind,axis,timeout=2) if payload is None else console.command(kind,axis,payload,timeout=2)
    if not r or r.get('kind')!=0x80 or r.get('status')!='ok':
        raise RuntimeError(f'Console error: {r}')
    return r


def status(console):
    # Read generation on each side of multi-packet fields to detect tearing.
    for _ in range(3):
        first=checked(console,0x31,7)['value']
        words=[checked(console,0x31,i)['value'] for i in range(10)]
        last=checked(console,0x31,7)['value']
        if first==last==words[7]:break
    else:raise RuntimeError('Capture state changed during snapshot')
    flags=words[0]
    return dict(background_valid=bool(flags&0x8000),templates_loaded=(flags>>13)&3,
                stable=bool(flags&0x1000),stable_class=(flags>>10)&3,
                record_busy=bool(flags&0x200),background_busy=bool(flags&0x100),
                capture_error=bool(flags&0x80),reject_reason=(flags>>4)&7,
                motion_valid=bool(flags&8),track_valid=bool(flags&4),global_change=bool(flags&2),
                cuboid_errors=words[1],wedge_errors=words[2],
                prediction_frame_id=(words[3]<<16)|words[4],
                capture_frame_id=(words[5]<<16)|words[6],generation=words[7],
                motion_cells=words[8],source_frame_low16=words[9])


def diagnostic(console):
    words=[checked(console,0x32,i)['value'] for i in range(4)]
    # Address0 above freezes all224 bits. Nonzero legacy addresses read that
    # same snapshot without freezing a different source frame.
    ph=checked(console,0x31,3)['value'];pl=checked(console,0x31,4)['value']
    source=checked(console,0x31,9)['value']
    flags=words[0];reason=flags&7
    return dict(threshold_rb=(flags>>10)&31,threshold_g=(flags>>4)&63,
                object_valid=bool(flags&8),mask_reject=reason,
                mask_reject_name={0:'usable',1:'empty',2:'too_few_pixels',3:'global_foreground',
                                  4:'too_narrow',5:'touches_roi_border'}.get(reason,'reserved'),
                foreground_pixels=words[1],raw_bbox=[words[2]>>8,words[3]>>8,words[2]&255,words[3]&255],
                prediction_frame_id=(ph<<16)|pl,source_frame_low16=source)



def storage_status(console):
    value=checked(console,0x37)['value']
    return dict(saved_templates=value&3,background_saved=bool(value&4),error=(value>>3)&7,busy=bool(value&64),
                flash_present=bool(value&128),capture_mode=bool(value&256),
                capture_notice=(value>>9)&7)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action',choices=['status','background','cuboid','wedge','diagnostic','threshold','storage-status'])
    ap.add_argument('--rb',type=int,default=4);ap.add_argument('--green',type=int,default=8)
    ap.add_argument('--port',default='FTDI:123456789B')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seconds',type=float,default=10)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    c=None;r={'action':args.action,'motor_commands_sent':False,'completed':False}
    try:
        c=Console(args.port);hello=checked(c,0x15)
        if hello['value'] not in (0x3915,0x3916,0x3917,0x3918,0x3919,0x391a,0x391b,0x391c,0x391d,0x391e,0x391f,0x3920,0x3921,0x3922) or hello['armed']:raise RuntimeError('Expected disarmed template console0x3915/0x3916/0x3917/0x3918/0x3919/0x391a/0x391b/0x391c')
        r['before']=status(c)
        if args.action=='storage-status':
            r['storage']=storage_status(c);r['completed']=True
        elif args.action=='diagnostic':
            r['diagnostic']=diagnostic(c);r['completed']=True
        elif args.action=='threshold':
            if not 1<=args.rb<=31 or not 1<=args.green<=63:raise ValueError('RB1..31, G1..63 required')
            if checked(c,0x1f)['value']!=0:raise RuntimeError('Manual axis release active')
            r['ack']=checked(c,0x33,0,bytes((args.rb,args.green))+bytes(10))
            deadline=time.monotonic()+args.seconds
            while time.monotonic()<deadline:
                time.sleep(.1);r['diagnostic']=diagnostic(c);r['after']=status(c)
                if (r['diagnostic']['threshold_rb'],r['diagnostic']['threshold_g'])==(args.rb,args.green) and r['after']['templates_loaded']==0:
                    r['completed']=True;break
            if not r['completed']:raise TimeoutError('Threshold ACK not applied at a source frame boundary')
        elif args.action!='status':
            if checked(c,0x1f)['value']!=0:raise RuntimeError('Manual axis release active')
            if args.action!='background' and not r['before']['background_valid']:
                raise RuntimeError('Capture empty background first')
            r['ack']=checked(c,0x30,{'background':1,'cuboid':2,'wedge':3}[args.action])
            deadline=time.monotonic()+args.seconds
            while time.monotonic()<deadline:
                time.sleep(.1);r['after']=status(c)
                if r['after']['generation']!=r['before']['generation']:
                    if r['after']['capture_error']:raise RuntimeError('No complete foreground object; inspect ROI on HDMI')
                    r['completed']=True;break
            if not r['completed']:raise TimeoutError('ACK received but capture did not complete')
        else:r['after']=r['before'];r['completed']=True
        if hello['value'] in (0x391b,0x391c,0x391d,0x391e,0x391f,0x3920,0x3921,0x3922) and args.action in ('background','cuboid','wedge'):
            r['completed']=False
            required={'background':4,'cuboid':1,'wedge':2}[args.action]
            while time.monotonic()<deadline:
                r['storage']=storage_status(c)
                if r['storage']['error']:
                    raise RuntimeError(f"Flash save failed: {r['storage']}")
                saved=r['storage']['saved_templates'] | (4 if r['storage']['background_saved'] else 0)
                if not r['storage']['busy'] and saved&required:
                    r['completed']=True;break
                time.sleep(.1)
            if not r['completed']:raise TimeoutError('Capture completed, but Flash save has not completed')
    except Exception as e:
        r['error']=str(e);raise
    finally:
        if c:c.close()
        (args.output/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(r,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
