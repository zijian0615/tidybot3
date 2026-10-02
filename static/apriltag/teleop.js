'use strict';
const token=window.TELEOP_TOKEN;
const statusEl=document.querySelector('#status'),tagStatus=document.querySelector('#tagStatus');
const pad=document.querySelector('#pad'),knob=document.querySelector('#knob');
const video=document.querySelector('#video'),overlay=document.querySelector('#overlay'),work=document.querySelector('#work');
const ctx=work.getContext('2d',{willReadFrequently:true}),octx=overlay.getContext('2d');
let x=0,y=0,rot=0,held=false,lastOK=0,baseSeq=0,poseSeq=1000000;
let detector=null,cameraRunning=false,detecting=false,lastTagAt=0,filtered=null;

function active(){return held||rot!==0}
function setPad(e){const r=pad.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;let dx=(e.clientX-cx)/(r.width*.4),dy=(e.clientY-cy)/(r.height*.4),m=Math.hypot(dx,dy);if(m>1){dx/=m;dy/=m}x=dx;y=-dy;knob.style.left=(50+dx*40)+'%';knob.style.top=(50+dy*40)+'%'}
async function post(path,data){try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});if(r.ok){lastOK=Date.now();const d=await r.json();statusEl.textContent=d.mode+' | '+d.state}}catch(e){statusEl.textContent='连接丢失；机器人应已停止'}}
function sendBase(){post('/cmd',{token,seq:++baseSeq,enabled:active(),x,y,rotation:rot})}
function stopArm(){filtered=null;post('/arm_tag_pose',{token,seq:++poseSeq,enabled:false,position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}})}
function stopBase(){held=false;x=0;y=0;rot=0;knob.style.left='50%';knob.style.top='50%';for(const id of ['left','right'])document.querySelector('#'+id).classList.remove('active');sendBase();stopArm()}
pad.onpointerdown=e=>{held=true;pad.setPointerCapture(e.pointerId);setPad(e);sendBase()};pad.onpointermove=e=>{if(held)setPad(e)};pad.onpointerup=stopBase;pad.onpointercancel=stopBase;
for(const [id,v] of [['left',1],['right',-1]]){const b=document.querySelector('#'+id);b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);rot=v;b.classList.add('active');sendBase()};b.onpointerup=b.onpointercancel=stopBase}

function solve(A,b){for(let i=0;i<b.length;i++){let p=i;for(let j=i+1;j<b.length;j++)if(Math.abs(A[j][i])>Math.abs(A[p][i]))p=j;[A[i],A[p]]=[A[p],A[i]];[b[i],b[p]]=[b[p],b[i]];if(Math.abs(A[i][i])<1e-9)throw Error('singular homography');let d=A[i][i];for(let k=i;k<A[i].length;k++)A[i][k]/=d;b[i]/=d;for(let j=0;j<b.length;j++)if(j!==i){let q=A[j][i];for(let k=i;k<A[j].length;k++)A[j][k]-=q*A[i][k];b[j]-=q*b[i]}}return b}
function homography(c){const obj=[[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]],A=[],b=[];for(let i=0;i<4;i++){const [X,Y]=obj[i],u=c[i].x,v=c[i].y;A.push([X,Y,1,0,0,0,-u*X,-u*Y]);b.push(u);A.push([0,0,0,X,Y,1,-v*X,-v*Y]);b.push(v)}return [...solve(A,b),1]}
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2],norm=a=>Math.sqrt(dot(a,a));
const scale=(a,s)=>a.map(v=>v*s),sub=(a,b)=>a.map((v,i)=>v-b[i]);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
function matrixQuat(R){const tr=R[0][0]+R[1][1]+R[2][2];let x,y,z,w;if(tr>0){let s=Math.sqrt(tr+1)*2;w=.25*s;x=(R[2][1]-R[1][2])/s;y=(R[0][2]-R[2][0])/s;z=(R[1][0]-R[0][1])/s}else if(R[0][0]>R[1][1]&&R[0][0]>R[2][2]){let s=Math.sqrt(1+R[0][0]-R[1][1]-R[2][2])*2;w=(R[2][1]-R[1][2])/s;x=.25*s;y=(R[0][1]+R[1][0])/s;z=(R[0][2]+R[2][0])/s}else if(R[1][1]>R[2][2]){let s=Math.sqrt(1+R[1][1]-R[0][0]-R[2][2])*2;w=(R[0][2]-R[2][0])/s;x=(R[0][1]+R[1][0])/s;y=.25*s;z=(R[1][2]+R[2][1])/s}else{let s=Math.sqrt(1+R[2][2]-R[0][0]-R[1][1])*2;w=(R[1][0]-R[0][1])/s;x=(R[0][2]+R[2][0])/s;y=(R[1][2]+R[2][1])/s;z=.25*s}return{x,y,z,w}}
function tagPose(c,w,h){const H=homography(c),fov=+document.querySelector('#fov').value*Math.PI/180,fx=w/(2*Math.tan(fov/2)),fy=fx,cx=w/2,cy=h/2;
  const kinv=v=>[(v[0]-cx*v[2])/fx,(v[1]-cy*v[2])/fy,v[2]];
  let h1=kinv([H[0],H[3],H[6]]),h2=kinv([H[1],H[4],H[7]]),h3=kinv([H[2],H[5],H[8]]),lam=2/(norm(h1)+norm(h2));
  let r1=scale(h1,lam),r2=scale(h2,lam);r1=scale(r1,1/norm(r1));r2=sub(r2,scale(r1,dot(r1,r2)));r2=scale(r2,1/norm(r2));let r3=cross(r1,r2),t=scale(h3,lam*(+document.querySelector('#tagSize').value/1000));
  if(t[2]<0){r1=scale(r1,-1);r2=scale(r2,-1);r3=cross(r1,r2);t=scale(t,-1)}
  const Rtc=[[r1[0],r2[0],r3[0]],[r1[1],r2[1],r3[1]],[r1[2],r2[2],r3[2]]],Rct=[[Rtc[0][0],Rtc[1][0],Rtc[2][0]],[Rtc[0][1],Rtc[1][1],Rtc[2][1]],[Rtc[0][2],Rtc[1][2],Rtc[2][2]]],cam=Rct.map(row=>-dot(row,t));
  // Proper rotation from tag coordinates to robot coordinates: robot X=-tag Z,
  // robot Y=tag X, robot Z=-tag Y. Direction signs can be tuned after preview.
  const B=[[0,0,-1],[1,0,0],[0,-1,0]],mul=(A,C)=>A.map(row=>[0,1,2].map(j=>dot(row,[C[0][j],C[1][j],C[2][j]]))),Bt=[[0,1,0],[0,0,-1],[-1,0,0]];
  const RR=mul(mul(B,Rct),Bt),p=[-cam[2],cam[0],-cam[1]];return{p,q:matrixQuat(RR)}}
function area(c){let s=0;for(let i=0;i<4;i++){const j=(i+1)%4;s+=c[i].x*c[j].y-c[j].x*c[i].y}return Math.abs(s)/2}
function drawTag(d,w,h){overlay.width=overlay.clientWidth*devicePixelRatio;overlay.height=overlay.clientHeight*devicePixelRatio;octx.clearRect(0,0,overlay.width,overlay.height);const sx=overlay.width/w,sy=overlay.height/h;octx.strokeStyle='#00e676';octx.lineWidth=4;octx.beginPath();d.corners.forEach((p,i)=>i?octx.lineTo(p.x*sx,p.y*sy):octx.moveTo(p.x*sx,p.y*sy));octx.closePath();octx.stroke()}
function onDetections(ds,ms){detecting=false;const d=ds.find(v=>v.family==='tag36h11'&&v.id===0&&v.hamming===0&&v.decisionMargin>=25&&area(v.corners)>=900);if(!d){if(Date.now()-lastTagAt>180){tagStatus.textContent='标记丢失；机械臂已停止';if(filtered)stopArm()}return}try{const pose=tagPose(d.corners,work.width,work.height),a=.28;if(!filtered)filtered=pose;else{filtered.p=filtered.p.map((v,i)=>v*(1-a)+pose.p[i]*a);let q=pose.q,old=filtered.q;if(q.x*old.x+q.y*old.y+q.z*old.z+q.w*old.w<0)q={x:-q.x,y:-q.y,z:-q.z,w:-q.w};let nq={x:old.x*(1-a)+q.x*a,y:old.y*(1-a)+q.y*a,z:old.z*(1-a)+q.z*a,w:old.w*(1-a)+q.w*a},n=Math.hypot(nq.x,nq.y,nq.z,nq.w);filtered.q={x:nq.x/n,y:nq.y/n,z:nq.z/n,w:nq.w/n}}lastTagAt=Date.now();drawTag(d,work.width,work.height);tagStatus.textContent=`Tag 0 | ${ms.toFixed(0)} ms | margin ${d.decisionMargin.toFixed(0)} | 距离 ${Math.hypot(...filtered.p).toFixed(2)} m`;post('/arm_tag_pose',{token,seq:++poseSeq,enabled:active(),position:{x:filtered.p[0],y:filtered.p[1],z:filtered.p[2]},orientation:filtered.q})}catch(e){tagStatus.textContent='位姿计算失败: '+e.message;stopArm()}}
async function startCamera(){if(!window.isSecureContext){tagStatus.textContent='Safari 相机要求 HTTPS；当前页面不是安全连接';return}document.querySelector('#startCamera').disabled=true;tagStatus.textContent='正在加载检测器…';detector=new AprilTagDetectorManager();await detector.initialize({onDetectionResult:onDetections,onError:e=>tagStatus.textContent=e});await detector.setupDetector('tag36h11',0,{quad_decimate:2,quad_sigma:0,refine_edges:1,decode_sharpening:.25});const stream=await navigator.mediaDevices.getUserMedia({video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480}},audio:false});video.srcObject=stream;await video.play();work.width=video.videoWidth;work.height=video.videoHeight;cameraRunning=true;tagStatus.textContent='寻找 tag36h11 ID 0';requestAnimationFrame(frame)}
function frame(){if(!cameraRunning)return;if(!detecting&&video.readyState>=2){ctx.drawImage(video,0,0,work.width,work.height);detecting=true;detector.detect(ctx.getImageData(0,0,work.width,work.height),work.width,work.height).catch(e=>{detecting=false;tagStatus.textContent=e.message})}requestAnimationFrame(frame)}
document.querySelector('#startCamera').onclick=()=>startCamera().catch(e=>{document.querySelector('#startCamera').disabled=false;tagStatus.textContent='相机启动失败: '+e.message});
for(const id of ['tagSize','fov']){const el=document.querySelector('#'+id),saved=localStorage.getItem(id);if(saved)el.value=saved;el.onchange=()=>localStorage.setItem(id,el.value)}
setInterval(sendBase,80);setInterval(()=>{if(Date.now()-lastOK>500)statusEl.textContent='连接超时；机器人应已停止'},200);
function stopAll(){stopBase()}document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll()});window.addEventListener('blur',stopAll);
