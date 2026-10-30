"""USB-UART diagnostics for the independent FPGA ZLink controller."""
from pathlib import Path
import argparse,json,struct,time,sys,ctypes,re
import serial
from serial.tools import list_ports
ROOT=Path(__file__).resolve().parents[1]
STATUS={0:'ok',1:'packet_crc_or_timeout',2:'unsupported',3:'busy',4:'not_armed',5:'invalid_parameter',6:'servo_timeout',8:'invalid_position',9:'position_mode_not_verified',10:'manual_release_active_or_not_disarmed',11:'manual_restore_alignment_failed'}
def crc8(data):
 c=0
 for b in data:
  c^=b
  for _ in range(8):c=((c<<1)^7)&255 if c&128 else (c<<1)&255
 return c
def packet(kind,id=0,payload=b'\0'*12):
 assert len(payload)==12
 body=bytes((kind,id))+payload
 return b'\xd5'+body+bytes((crc8(body),))
def ports():
 return [p.device for p in list_ports.comports() if p.vid==0x0403 and p.pid==0x6042]
class FtdiSerial:
 """Use FTDI channel B directly when Windows COM aliases collide."""
 def __init__(self,serial_number):
  self.dll=ctypes.WinDLL('C:/Windows/System32/ftd2xx.dll');self.handle=ctypes.c_void_p()
  self.check(self.dll.FT_OpenEx(ctypes.c_char_p(serial_number.encode('ascii')),1,ctypes.byref(self.handle)))
  try:
   for name,args in [('FT_SetBaudRate',(115200,)),('FT_SetDataCharacteristics',(8,0,0)),('FT_SetFlowControl',(0,0,0)),('FT_SetTimeouts',(50,1000)),('FT_SetLatencyTimer',(2,))]:
    self.check(getattr(self.dll,name)(self.handle,*args))
  except Exception:self.close();raise
 def check(self,status):
  if status:raise RuntimeError(f'FTDI status {status}')
 def reset_input_buffer(self):self.check(self.dll.FT_Purge(self.handle,1))
 @property
 def in_waiting(self):
  count=ctypes.c_ulong();self.check(self.dll.FT_GetQueueStatus(self.handle,ctypes.byref(count)));return count.value
 def read(self,count):
  data=ctypes.create_string_buffer(count);actual=ctypes.c_ulong()
  self.check(self.dll.FT_Read(self.handle,data,count,ctypes.byref(actual)));return data.raw[:actual.value]
 def write(self,data):
  actual=ctypes.c_ulong();self.check(self.dll.FT_Write(self.handle,ctypes.c_char_p(data),len(data),ctypes.byref(actual)))
  if actual.value!=len(data):raise RuntimeError('Incomplete FTDI write')
 def flush(self):pass
 def close(self):
  if self.handle:self.dll.FT_Close(self.handle);self.handle=ctypes.c_void_p()
class Console:
 def __init__(self,port):
  if port.startswith('FTDI:'):self.serial=FtdiSerial(port[5:])
  else:
   self.serial=serial.Serial(port=None,baudrate=115200,timeout=.05,write_timeout=1)
   self.serial.port=port;self.serial.dtr=False;self.serial.rts=False;self.serial.open()
  self.buffer=bytearray();self.raw_frames=[];self.serial.reset_input_buffer()
 def close(self):self.serial.close()
 def receive(self,timeout):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   self.buffer+=self.serial.read(max(1,self.serial.in_waiting))
   while len(self.buffer)>=16:
    if self.buffer[0]!=0xd5:del self.buffer[0];continue
    raw=bytes(self.buffer[:16])
    if crc8(raw[1:15])!=raw[15]:del self.buffer[0];continue
    del self.buffer[:16]
    kind,address=raw[1:3];request,id=raw[3:5]
    if kind in (0x84,0x85,0x86,0x87):
     capture=dict(kind=kind,address=address,request=0x1a if kind in (0x85,0x86) else 0xff,id=0 if kind in (0x85,0x86) else 0xff,
                  ascii=raw[3:3+min(address,12)].decode('ascii',errors='replace'),raw=raw.hex())
     self.raw_frames.append(capture)
     return capture
    value,rx,tx,errors=struct.unpack('>HHHH',raw[5:13])
    result=dict(kind=kind,address=address,request=request,id=id,value=value,rx_bytes=rx,tx_commands=tx,errors=errors,armed=bool(raw[13]),target_reached=bool(raw[14]&1),raw=raw.hex())
    if kind==0x80:result['status']=STATUS.get(address,f'error_{address}')
    if request in (0x1b,0x1c,0x1d):
     result['capture_may_be_truncated']=bool(raw[14]&1)
     del result['target_reached']
    if request==0x16:
     result.update(local_tx_bytes=value,rx_edges=tx,rx_level=raw[13]&1,tx_level=(raw[13]>>1)&1,state=raw[14])
     del result['armed'];del result['target_reached'];del result['tx_commands']
    return result
  return None
 def command(self,kind,id=0,payload=b'\0'*12,timeout=1):
  self.serial.write(packet(kind,id,payload));self.serial.flush()
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   r=self.receive(end-time.monotonic())
   if r is None:return None
   if r['request']==kind and (r['id']==id or kind in (0x12,0x14)):return r
  return None
def parse_health(ascii_reply,expected_id):
 observed=re.search(r'#(\d{3})@T(-?\d{2,3})V(\d{3})!',ascii_reply)
 if observed and int(observed[1])==expected_id:
  return dict(format='ZServo_at_observed',servo_id=expected_id,
              temperature_code=observed[2],temperature_unit_confirmed=False,
              voltage_code=int(observed[3]),voltage_v_estimate=int(observed[3])/10,
              voltage_unit_confirmed=False,
              interpretation='Tenths-of-volt hypothesis awaiting manual or multimeter validation')
 legacy=re.search(r'#(\d{3})T(\d{4})-(\d{2}\.\d)!',ascii_reply)
 modern=re.search(r'#(\d{3})T(\d{1,3})V(\d{1,2}(?:\.\d+)?)!',ascii_reply)
 if legacy and int(legacy[1])==expected_id and 0<=int(legacy[2])<=4095:
  ad=int(legacy[2]);resistance=10*ad/2048
  return dict(format='PRTE_AD',servo_id=expected_id,temperature_ad=ad,
              temperature_c=round(.633916*resistance**2-13.2675*resistance+94.28351,2),
              voltage_v=float(legacy[3]),voltage_resolution_v=.1)
 if modern and int(modern[1])==expected_id:
  return dict(format='PRTV',servo_id=expected_id,temperature_c=int(modern[2]),
              voltage_v=float(modern[3]),voltage_resolution_v=.1 if '.' in modern[3] else 1.)
 return None

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('action',choices=('hello','manual-status','scan','scan-id','scan-mode','scan-health','health','version','trace','mode','ping','read','diagnostic','loopback','watch','arm','disable','stop','restore','release','jog','sort-start','sort-status'))
 p.add_argument('--port');p.add_argument('--id',type=int,default=5);p.add_argument('--delta',type=int,default=20);p.add_argument('--time-ms',type=int,default=2000)
 p.add_argument('--seconds',type=float,default=30);p.add_argument('--output',type=Path);p.add_argument('--restore-trace',action='store_true');args=p.parse_args()
 available=ports();port=args.port or (available[0] if len(available)==1 else None)
 if port is None:raise SystemExit(f'Select the board FTDI-B UART explicitly. Detected ports: {available}')
 if not 0<=args.id<=5:raise SystemExit('ID must be 0..5')
 if args.action=='jog' and not(0<abs(args.delta)<=150 and 500<=args.time_ms<=5000):raise SystemExit('Jog must be within +/-150 and 500..5000ms')
 c=Console(port);records=[]
 try:
  hello=c.command(0x15)
  if hello is None or hello['kind']!=0x80 or hello['value'] not in (0x3910,0x3911,0x3912,0x3913,0x3914,0x3915,0x3916,0x3917,0x3918,0x3919,0x391a,0x391b,0x391c,0x391d,0x391e,0x391f,0x3920,0x3921,0x3922):raise RuntimeError(f'FPGA console signature not received: {hello}')
  print(json.dumps(dict(port=port,hello=hello)),flush=True);records.append(hello)
  if args.action in ('sort-start','sort-status'):
   r=c.command(0x35 if args.action=='sort-start' else 0x36);record=dict(response=r)
   if args.action=='sort-status' and r and r.get('status')=='ok':
    record.update(phase=(r['value']>>8)&31,step=(r['value']>>3)&31,part=(r['value']>>1)&3,active=bool(r['value']&1))
   records.append(record);print(json.dumps(record),flush=True)
  elif args.action=='manual-status':
   r=c.command(0x1f);records.append(r);print(json.dumps(dict(response=r,manual_axes=[i for i in range(6) if r['value']&(1<<i)] if r and r.get('status')=='ok' else None)),flush=True)
  elif args.action in ('health','scan-health','version'):
   for id in (range(6) if args.action=='scan-health' else [args.id]):
    for name,kind in ([('PVER',0x1d)] if args.action=='version' else [('PRTE',0x1b),('PRTV',0x1c)]):
     c.raw_frames.clear();r=c.command(kind,id);frames=list(c.raw_frames)
     ascii_reply=''.join(frame['ascii'] for frame in frames)
     record=dict(id=id,query=name,response=r,ascii=ascii_reply,captured_frames=frames,
                 health=parse_health(ascii_reply,id) if args.action!='version' else None)
     records.append(record);print(json.dumps(record),flush=True)
  elif args.action=='trace':
   c.raw_frames.clear();first=c.command(0x1a,0,bytes([int(args.restore_trace)])+b'\0'*11)
   tail=c.receive(.5) if first and first.get('kind')==0x85 else None
   record=dict(first=first,tail=tail,ascii=''.join(r.get('ascii','') for r in (first,tail) if r))
   records.append(record);print(json.dumps(record),flush=True)
  elif args.action in ('scan','scan-id','scan-mode'):
   for id in range(6):
    r=c.command({'scan-id':0x17,'scan-mode':0x18,'scan':0x10}[args.action],id);record=dict(id=id,response=r)
    if args.action=='scan-mode':record['captured_frames']=list(c.raw_frames);c.raw_frames.clear()
    records.append(record);print(json.dumps(record),flush=True)
  elif args.action=='mode':
   r=c.command(0x18,args.id);record=dict(response=r,captured_frames=c.raw_frames);records.append(record);print(json.dumps(record),flush=True)
  elif args.action=='ping':
   r=c.command(0x17,args.id);records.append(r);print(json.dumps(r),flush=True)
  elif args.action=='read':
   r=c.command(0x10,args.id);records.append(r);print(json.dumps(r),flush=True)
  elif args.action=='diagnostic':
   before=c.command(0x16);records.append(before)
   query=c.command(0x10,args.id);records.append(query)
   after=c.command(0x16);records.append(after)
   result=dict(test='gpio_uart_diagnostic',before=before,query=query,after=after)
   if before and after and before['status']=='ok' and after['status']=='ok':
    for field in ('local_tx_bytes','rx_bytes','rx_edges','errors'):
     result[field+'_delta']=(after[field]-before[field])&65535
   print(json.dumps(result),flush=True);records.append(result)
  elif args.action=='loopback':
   r=c.command(0x10,args.id);records.append(r)
   after=c.command(0x15);records.append(after)
   result=dict(test='translated_uart_loopback',query_response=r,after=after)
   if after:
    result.update(rx_delta=(after['rx_bytes']-hello['rx_bytes'])&65535,
                  error_delta=(after['errors']-hello['errors'])&65535)
    result['nine_uart_bytes_received']=result['rx_delta']==9 and result['error_delta']==0
   print(json.dumps(result),flush=True);records.append(result)
  elif args.action=='release':
   if hello['value'] not in (0x3913,0x3914,0x3915,0x3916,0x3917,0x3918,0x3919,0x391a,0x391b,0x391c,0x391d,0x391e,0x391f,0x3920,0x3921,0x3922):raise RuntimeError('Release requires qualified manual-calibration firmware')
   if hello['armed']:raise RuntimeError('Controller must be disarmed before manual release')
   mode=c.command(0x18,args.id)
   if not mode or mode.get('kind')!=0x83 or mode.get('value') not in (1,2,3,4):raise RuntimeError('Confirmed position mode required')
   records.append(mode)
   r=c.command(0x1e,args.id,b'FREE'+b'\0'*8)
   records.append(r);print(json.dumps(dict(release_tx_ack=r,physical_release_verified=False)),flush=True)
  elif args.action in ('arm','disable','stop','jog','restore'):
   kind=dict(arm=0x13,disable=0x14,stop=0x12,jog=0x11,restore=0x19)[args.action]
   payload=b'ARM!'+b'\0'*8 if args.action=='arm' else struct.pack('>hH',args.delta,args.time_ms)+b'\0'*8 if args.action=='jog' else b'\0'*12
   r=c.command(kind,args.id if args.action in ('jog','restore') else 0,payload,timeout=args.time_ms/1000+2 if args.action=='jog' else 5 if args.action=='restore' else 1)
   records.append(r);print(json.dumps(r),flush=True)
  elif args.action=='watch':
   end=time.monotonic()+args.seconds
   while time.monotonic()<end:
    r=c.receive(min(.5,end-time.monotonic()))
    if r:records.append(r);print(json.dumps(r),flush=True)
 finally:
  c.close()
  if args.output:
   args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(dict(port=port,records=records),indent=2))
if __name__=='__main__':main()
