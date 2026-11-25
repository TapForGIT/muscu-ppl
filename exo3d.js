// =====================================================================
//  Muscu PPL — visuels 3D des exercices (Three.js)
//  Chaque exercice = build(scene) -> { pose(t) } ; t = 0 départ, 1 fin de la phase concentrique.
//  Le mannequin est piloté par des points (bassin, orientation du buste, mains, pieds) ;
//  bras et jambes sont calculés par cinématique inverse (coudes/genoux orientés par un "pole").
// =====================================================================
(function(){
const D = Math.PI/180;
const V = (x,y,z)=>new THREE.Vector3(x,y,z);
const lerp=(a,b,t)=>a+(b-a)*t;
const MAT = {
  body: new THREE.MeshStandardMaterial({color:0xb7b7b7, roughness:0.55, metalness:0.05}),
  steel: new THREE.MeshStandardMaterial({color:0x50545a, roughness:0.35, metalness:0.6}),
  dark: new THREE.MeshStandardMaterial({color:0x2a2a2a, roughness:0.7}),
  pad: new THREE.MeshStandardMaterial({color:0x7a3b2e, roughness:0.85}),
  floor: new THREE.MeshStandardMaterial({color:0x8f8f8f, roughness:0.95}),
  cable: new THREE.MeshStandardMaterial({color:0x1c1c1c, roughness:0.6}),
  rope: new THREE.MeshStandardMaterial({color:0x3a4a5a, roughness:0.8})
};
function cyl(r1,r2,len,m,seg=20){ return new THREE.Mesh(new THREE.CylinderGeometry(r1,r2,len,seg), m); }
function sph(r,m){ return new THREE.Mesh(new THREE.SphereGeometry(r,22,16), m); }
function box(x,y,z,m){ return new THREE.Mesh(new THREE.BoxGeometry(x,y,z), m); }
function between(mesh,a,b){ const d=new THREE.Vector3().subVectors(b,a), l=Math.max(1e-4,d.length());
  mesh.position.copy(a).addScaledVector(d,0.5); mesh.scale.set(1,l,1);
  mesh.quaternion.setFromUnitVectors(V(0,1,0), d.normalize()); mesh.userData.a=a.clone(); mesh.userData.b=b.clone(); }
function rod(scene,r,m){ const c=cyl(r,r,1,m,14); scene.add(c); return c; }
function cable(scene){ const c=rod(scene,0.006,MAT.cable); c.userData.cable=true; return c; }
function fixed(scene,r,a,b,m){ const c=rod(scene,r,m||MAT.steel); between(c,a,b); return c; }
function basisQuat(right,up,fwd){ return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(right,up,fwd)); }
function ik3(A,B,l1,l2,pole){
  const AB=new THREE.Vector3().subVectors(B,A); let d=AB.length(); d=Math.min(Math.max(d,Math.abs(l1-l2)+1e-3), l1+l2-1e-3);
  const dir=AB.normalize(); const a=(l1*l1-l2*l2+d*d)/(2*d), h=Math.sqrt(Math.max(0,l1*l1-a*a));
  const p=pole.clone().addScaledVector(dir,-pole.dot(dir)); if(p.lengthSq()<1e-6) p.set(0,0,1); p.normalize();
  return A.clone().addScaledVector(dir,a).addScaledVector(p,h);
}
// direction interpolée sur la sphère (trajectoires en arc)
function slerpDir(a,b,t){ const qa=new THREE.Quaternion().setFromUnitVectors(a.clone().normalize(), b.clone().normalize()); const q=new THREE.Quaternion().slerp(qa,t); return a.clone().normalize().applyQuaternion(q); }
const BODY={ua:0.29, fa:0.30, th:0.43, sh:0.42};

// ---------------- Mannequin ----------------
function Man(scene){
  const torso=new THREE.Group(); scene.add(torso);
  const hipM=sph(0.15,MAT.body); hipM.scale.set(1.15,0.75,0.8); torso.add(hipM);
  const abd=cyl(0.13,0.14,0.26,MAT.body); abd.position.y=0.15; abd.scale.z=0.75; torso.add(abd);
  const chest=sph(0.19,MAT.body); chest.position.y=0.4; chest.scale.set(1.08,1.05,0.72); torso.add(chest);
  const neck=cyl(0.05,0.055,0.12,MAT.body); neck.position.y=0.6; torso.add(neck);
  const head=sph(0.11,MAT.body); head.position.set(0,0.73,0.01); head.scale.set(0.9,1.08,1); torso.add(head);
  const parts={}; const caps=[];
  for(const k of ['L','R']){
    parts['sh'+k]=sph(0.072,MAT.body); parts['el'+k]=sph(0.048,MAT.body); parts['hd'+k]=sph(0.045,MAT.body);
    parts['ua'+k]=cyl(0.045,0.058,1,MAT.body); parts['fa'+k]=cyl(0.034,0.045,1,MAT.body);
    parts['hp'+k]=sph(0.085,MAT.body); parts['kn'+k]=sph(0.06,MAT.body);
    parts['th'+k]=cyl(0.06,0.085,1,MAT.body); parts['sn'+k]=cyl(0.042,0.058,1,MAT.body);
    parts['ft'+k]=box(0.09,0.06,0.22,MAT.body);
    for(const n of ['sh','el','hd','ua','fa','hp','kn','th','sn','ft']) scene.add(parts[n+k]);
  }
  const J={};
  function set(P){
    const up=P.up.clone().normalize(), fwd=P.fwd.clone().normalize(), right=new THREE.Vector3().crossVectors(up,fwd).normalize();
    const f2=new THREE.Vector3().crossVectors(right,up).normalize();
    torso.position.copy(P.pelvis); torso.quaternion.copy(basisQuat(right,up,f2)); torso.updateMatrixWorld(true);
    J.up=up; J.fwd=f2; J.right=right; J.pelvis=P.pelvis.clone();
    J.neck=torso.localToWorld(V(0,0.55,0)); J.head=torso.localToWorld(V(0,0.73,0.01)); J.chest=torso.localToWorld(V(0,0.38,0)); J.abd=torso.localToWorld(V(0,0.12,0));
    for(const [k,s] of [['L',-1],['R',1]]){
      const S=torso.localToWorld(V(0.2*s,0.5,0)), H=P.hand[k];
      const E=P.elbow&&P.elbow[k] ? P.elbow[k] : ik3(S,H,BODY.ua,BODY.fa,P.elbowPole[k]);
      parts['sh'+k].position.copy(S); parts['el'+k].position.copy(E); parts['hd'+k].position.copy(H);
      between(parts['ua'+k],S,E); between(parts['fa'+k],E,H);
      const Hp=torso.localToWorld(V(0.1*s,-0.04,0)), A=P.foot[k];
      const K=ik3(Hp,A,BODY.th,BODY.sh,P.kneePole[k]);
      parts['hp'+k].position.copy(Hp); parts['kn'+k].position.copy(K);
      between(parts['th'+k],Hp,K); between(parts['sn'+k],K,A);
      const fd=(P.footDir&&P.footDir[k])?P.footDir[k].clone().normalize():V(0,0,1);
      const ref=Math.abs(fd.y)>0.95?V(0,0,1):V(0,1,0);
      const fr=new THREE.Vector3().crossVectors(ref,fd).normalize(), fu=new THREE.Vector3().crossVectors(fd,fr).normalize();
      parts['ft'+k].quaternion.copy(basisQuat(fr,fu,fd));
      parts['ft'+k].position.copy(A).addScaledVector(fd,0.06).addScaledVector(fu,-0.035);
      Object.assign(J,{['S'+k]:S,['E'+k]:E,['H'+k]:H.clone(),['Hp'+k]:Hp,['K'+k]:K,['A'+k]:A.clone()});
    }
    return J;
  }
  // capsules approximant le corps (contrôle des collisions)
  function capsules(){ const c=[{a:J.pelvis,b:J.chest,r:0.15,n:'buste'},{a:J.head,b:J.head,r:0.115,n:'tête'},{a:J.chest,b:J.neck,r:0.13,n:'poitrine'}];
    for(const k of ['L','R']){ c.push({a:J['S'+k],b:J['E'+k],r:0.05,n:'bras'+k},{a:J['E'+k],b:J['H'+k],r:0.04,n:'avant-bras'+k},{a:J['Hp'+k],b:J['K'+k],r:0.075,n:'cuisse'+k},{a:J['K'+k],b:J['A'+k],r:0.05,n:'tibia'+k}); }
    return c; }
  return {set, capsules, J};
}

// ---------------- Matériel ----------------
function floor(scene){ const f=new THREE.Mesh(new THREE.CircleGeometry(2.0,48),MAT.floor); f.rotation.x=-Math.PI/2; scene.add(f); }
function dumbbell(scene){ const g=new THREE.Group(); const h=cyl(0.014,0.014,0.14,MAT.steel); g.add(h);
  for(const y of [-0.085,0.085]){ const p=cyl(0.055,0.055,0.04,MAT.dark); p.position.y=y; g.add(p); } scene.add(g); return g; }
function placeDB(db,pos,axis){ db.position.copy(pos); db.quaternion.setFromUnitVectors(V(0,1,0),axis.clone().normalize()); }
// poste à poulie : montant + poulie à la hauteur h, au point (x,z)
function tower(scene,x,z,h,stackSide){ fixed(scene,0.04,V(x,0,z),V(x,2.15,z)); const st=box(0.24,0.9,0.18,MAT.dark); st.position.set(x+(stackSide||0),0.47,z); scene.add(st);
  const p=cyl(0.05,0.05,0.03,MAT.steel); p.rotation.z=Math.PI/2; p.position.set(x,h,z); scene.add(p); return V(x,h,z); }
// banc réglable : assise + dossier incliné (angle en degrés, 0 = plat). Tête vers -z.
function bench(scene,incl,hinge){ hinge=hinge||V(0,0.44,0);
  const seat=box(0.30,0.07,0.40,MAT.pad); seat.position.set(hinge.x,hinge.y,hinge.z+0.2); scene.add(seat);
  const g=new THREE.Group(); g.position.copy(hinge); g.rotation.x=incl*D; scene.add(g);
  const back=box(0.30,0.07,0.85,MAT.pad); back.position.set(0,0,-0.43); g.add(back);
  fixed(scene,0.035,V(hinge.x,hinge.y-0.04,hinge.z+0.15),V(hinge.x,0,hinge.z+0.15));
  g.updateMatrixWorld(true); const top=g.localToWorld(V(0,0,-0.7));
  if(incl>5) fixed(scene,0.03,top.clone().add(V(0,-0.04,0)),V(top.x,0,top.z)); else fixed(scene,0.035,V(hinge.x,hinge.y-0.04,hinge.z-0.6),V(hinge.x,0,hinge.z-0.6));
  const f=box(0.42,0.04,1.3,MAT.steel); f.position.set(hinge.x,0.02,hinge.z-0.2); scene.add(f);
  return g; }
// repère d'une personne allongée/assise sur le dossier incliné : up = vers la tête, fwd = côté poitrine
function benchFrame(incl,hinge){ const a=incl*D; const up=V(0,Math.sin(a),-Math.cos(a)), fwd=V(0,Math.cos(a),Math.sin(a));
  const pelvis=hinge.clone().addScaledVector(up,0.10).addScaledVector(fwd,0.15); return {up,fwd,pelvis}; }
function smithFrame(scene,zb){ for(const x of [-0.62,0.62]){ fixed(scene,0.035,V(x,0,zb),V(x,2.0,zb)); const b=box(0.08,0.04,0.9,MAT.steel); b.position.set(x,0.02,zb); scene.add(b); }
  fixed(scene,0.035,V(-0.62,2.0,zb),V(0.62,2.0,zb));
  const bar=rod(scene,0.016,MAT.steel), car=[box(0.07,0.1,0.07,MAT.steel),box(0.07,0.1,0.07,MAT.steel)]; car.forEach(c=>scene.add(c));
  const plates=[]; for(const x of [-0.74,-0.79,0.74,0.79]){ const p=cyl(0.2,0.2,0.04,MAT.dark); p.rotation.z=Math.PI/2; p.userData.x=x; scene.add(p); plates.push(p); }
  return function(y){ between(bar,V(-0.92,y,zb),V(0.92,y,zb)); car[0].position.set(-0.62,y,zb); car[1].position.set(0.62,y,zb); plates.forEach(p=>p.position.set(p.userData.x,y,zb)); }; }
function tiltVecs(deg){ const a=deg*D; return {up:V(0,Math.cos(a),Math.sin(a)), fwd:V(0,-Math.sin(a),Math.cos(a))}; }
const STAND={pelvis:V(0,0.93,0), up:V(0,1,0), fwd:V(0,0,1)};
const feetStand=(w,z)=>({L:V(-(w||0.14),0.07,z||0),R:V(w||0.14,0.07,z||0)});
const kneeFwd={L:V(-0.15,0,1),R:V(0.15,0,1)};
const toesOut={L:V(-0.15,0,1),R:V(0.15,0,1)};

// =====================================================================
//  EXERCICES (clé = nom exact dans l'appli)
// =====================================================================
const EX={};

EX["Rowing T poitrine appuyée prise large"]={ cam:{target:V(0,0.75,0),dist:3.1,h:1.0,ang:62}, build(scene){
  floor(scene); const man=Man(scene); const pelvis=V(0,0.95,-0.35), T=tiltVecs(58);
  const right=new THREE.Vector3().crossVectors(T.up,T.fwd);
  const padG=new THREE.Group(); padG.position.copy(pelvis); padG.quaternion.copy(basisQuat(right,T.up,T.fwd)); scene.add(padG);
  const pad=box(0.34,0.36,0.06,MAT.pad); pad.position.set(0,0.245,0.17); padG.add(pad); padG.updateMatrixWorld(true);
  const padBack=padG.localToWorld(V(0,0.2,0.21));
  for(const x of [-0.17,0.17]) fixed(scene,0.03,padBack.clone().add(V(x,0,0)),V(x,0,padBack.z));
  const fp=box(0.46,0.04,0.32,MAT.steel); fp.position.set(0,0.40,-0.68); fp.rotation.x=-22*D; scene.add(fp);
  fixed(scene,0.03,V(0,0.38,-0.68),V(0,0,-0.68)); const base=box(0.5,0.05,1.7,MAT.steel); base.position.set(0,0.025,0); scene.add(base);
  const P0=V(0,0.12,-0.60), L=0.98, R=0.32;
  const lever=rod(scene,0.03,MAT.steel), riser=rod(scene,0.028,MAT.steel), bar=rod(scene,0.02,MAT.steel), sleeve=rod(scene,0.026,MAT.steel);
  const pv=sph(0.05,MAT.steel); pv.position.copy(P0); scene.add(pv);
  const plates=[cyl(0.17,0.17,0.045,MAT.dark),cyl(0.17,0.17,0.045,MAT.dark)]; plates.forEach(p=>scene.add(p));
  const grips=[rod(scene,0.022,MAT.dark),rod(scene,0.022,MAT.dark)];
  return { man, pose(t){
    const phi=lerp(12.2,32.8,t)*D, dir=V(0,Math.sin(phi),Math.cos(phi)), nrm=V(0,Math.cos(phi),-Math.sin(phi));
    const E=P0.clone().addScaledVector(dir,L), G=E.clone().addScaledVector(nrm,R);
    between(lever,P0,E.clone().addScaledVector(dir,0.08)); between(riser,E,G);
    const gL=G.clone().add(V(-0.27,0,0)), gR=G.clone().add(V(0.27,0,0));
    between(bar,G.clone().add(V(-0.33,0,0)),G.clone().add(V(0.33,0,0)));
    between(grips[0],gL.clone().add(V(-0.06,0,0)),gL.clone().add(V(0.06,0,0))); between(grips[1],gR.clone().add(V(-0.06,0,0)),gR.clone().add(V(0.06,0,0)));
    between(sleeve,E.clone().addScaledVector(dir,0.08),E.clone().addScaledVector(dir,0.42));
    plates.forEach((p,i)=>{ p.position.copy(E).addScaledVector(dir,0.22+i*0.05); p.quaternion.setFromUnitVectors(V(0,1,0),dir); });
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:gL,R:gR}, foot:{L:V(-0.12,0.47,-0.70),R:V(0.12,0.47,-0.70)},
      elbowPole:{L:V(-1,0.5,-0.7),R:V(1,0.5,-0.7)}, kneePole:{L:V(0,-0.3,1),R:V(0,-0.3,1)}, footDir:{L:V(0,0.38,0.92),R:V(0,0.38,0.92)}});
  }};
}};

EX["Développé couché Smith"]={ cam:{target:V(0,0.7,0),dist:3.3,h:1.5,ang:55}, build(scene){
  floor(scene); const man=Man(scene); const hinge=V(0,0.44,0.40); bench(scene,0,hinge); const F=benchFrame(0,hinge);
  const zb=F.pelvis.z-0.42; const setBar=smithFrame(scene,zb);
  return { man, pose(t){ const y=lerp(1.10,0.775,t); setBar(y);
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:{L:V(-0.37,y,zb),R:V(0.37,y,zb)}, foot:{L:V(-0.26,0.07,F.pelvis.z+0.50),R:V(0.26,0.07,F.pelvis.z+0.50)},
      elbowPole:{L:V(-1,-0.9,0.25),R:V(1,-0.9,0.25)}, kneePole:{L:V(-0.2,1,0.2),R:V(0.2,1,0.2)}, footDir:toesOut}); }};
}};

EX["Développé incliné Smith"]={ cam:{target:V(0,0.85,0),dist:3.4,h:1.55,ang:55}, build(scene){
  floor(scene); const man=Man(scene); const inc=35, hinge=V(0,0.44,0.35); bench(scene,inc,hinge); const F=benchFrame(inc,hinge);
  const upper=F.pelvis.clone().addScaledVector(F.up,0.33).addScaledVector(F.fwd,0.14); const zb=upper.z; const setBar=smithFrame(scene,zb);
  const S=F.pelvis.clone().addScaledVector(F.up,0.5); const yb=upper.y+0.03, yt=S.y+Math.sqrt(0.585*0.585-Math.pow(zb-S.z,2)-0.17*0.17);
  return { man, pose(t){ const y=lerp(yt,yb,t); setBar(y);
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:{L:V(-0.37,y,zb),R:V(0.37,y,zb)}, foot:{L:V(-0.26,0.07,F.pelvis.z+0.50),R:V(0.26,0.07,F.pelvis.z+0.50)},
      elbowPole:{L:V(-1,-0.8,0.5),R:V(1,-0.8,0.5)}, kneePole:{L:V(-0.2,1,0.2),R:V(0.2,1,0.2)}, footDir:toesOut}); }};
}};

EX["Développé incliné haltères"]={ cam:{target:V(0,0.8,0),dist:3.0,h:1.5,ang:50}, build(scene){
  floor(scene); const man=Man(scene); const inc=35, hinge=V(0,0.44,0.25); bench(scene,inc,hinge); const F=benchFrame(inc,hinge);
  const C=F.pelvis.clone().addScaledVector(F.up,0.5), dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){
    const h={}; for(const [k,s] of [['L',-1],['R',1]]){ h[k]=C.clone().add(V(s*lerp(0.33,0.11,t),0,0)).addScaledVector(F.fwd,lerp(0.15,0.575,t)).addScaledVector(F.up,lerp(-0.12,0.0,t)); }
    placeDB(dbs[0],h.L,V(1,0,0)); placeDB(dbs[1],h.R,V(1,0,0));
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:h, foot:{L:V(-0.26,0.07,F.pelvis.z+0.50),R:V(0.26,0.07,F.pelvis.z+0.50)},
      elbowPole:{L:V(-1,0,0).addScaledVector(F.fwd,-1.0).addScaledVector(F.up,-0.6),R:V(1,0,0).addScaledVector(F.fwd,-1.0).addScaledVector(F.up,-0.6)},
      kneePole:{L:V(-0.2,1,0.2),R:V(0.2,1,0.2)}, footDir:toesOut}); }};
}};

EX["Écartés poulie haute vers basse"]={ cam:{target:V(0,1.1,0),dist:3.6,h:1.45,ang:28}, build(scene){
  floor(scene); const man=Man(scene); const pL=tower(scene,-1.0,-0.25,1.95,-0.15), pR=tower(scene,1.0,-0.25,1.95,0.15);
  fixed(scene,0.035,V(-1,2.15,-0.25),V(1,2.15,-0.25)); const c=[cable(scene),cable(scene)], hd=[rod(scene,0.02,MAT.dark),rod(scene,0.02,MAT.dark)];
  const T=tiltVecs(10), pelvis=V(0,0.92,0.05);
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const h={};
    for(const [k,s,i] of [['L',-1,0],['R',1,1]]){ const S=C.clone().add(V(0.2*s,0,0)); const d=slerpDir(V(s*0.97,0.05,0.2),V(-s*0.28,-0.62,0.73),t); h[k]=S.clone().addScaledVector(d,0.56);
      between(c[i],i?pR:pL,h[k]); between(hd[i],h[k].clone().add(V(0,-0.05,0)),h[k].clone().add(V(0,0.05,0))); }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.15,0.07,0.15),R:V(0.15,0.07,-0.15)},
      elbowPole:{L:V(-0.4,0.4,-1),R:V(0.4,0.4,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Écartés poulie basse vers haute"]={ cam:{target:V(0,1.0,0),dist:3.6,h:1.3,ang:28}, build(scene){
  floor(scene); const man=Man(scene); const pL=tower(scene,-1.0,-0.25,0.15,-0.15), pR=tower(scene,1.0,-0.25,0.15,0.15);
  fixed(scene,0.035,V(-1,2.15,-0.25),V(1,2.15,-0.25)); const c=[cable(scene),cable(scene)], hd=[rod(scene,0.02,MAT.dark),rod(scene,0.02,MAT.dark)];
  const T=tiltVecs(4), pelvis=V(0,0.92,0.08);
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const h={};
    for(const [k,s,i] of [['L',-1,0],['R',1,1]]){ const S=C.clone().add(V(0.2*s,0,0)); const d=slerpDir(V(s*0.55,-0.83,0.12),V(-s*0.22,-0.05,0.97),t); h[k]=S.clone().addScaledVector(d,0.56);
      between(c[i],i?pR:pL,h[k]); between(hd[i],h[k].clone().add(V(0,-0.05,0)),h[k].clone().add(V(0,0.05,0))); }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.15,0.07,0.12),R:V(0.15,0.07,-0.12)},
      elbowPole:{L:V(-1,-0.6,-0.4),R:V(1,-0.6,-0.4)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Élévations latérales poulie"]={ cam:{target:V(-0.05,1.0,0),dist:3.4,h:1.25,ang:20}, build(scene){
  floor(scene); const man=Man(scene); const P=tower(scene,-0.7,0.0,0.16,-0.2); const pul=V(-0.62,0.16,0.34); fixed(scene,0.02,V(-0.7,0.16,0),pul);
  const c=cable(scene), hd=rod(scene,0.02,MAT.dark);
  return { man, pose(t){ const S=V(0.2,1.43,0), al=lerp(-14,84,t)*D, r=0.575, sc=20*D;
    const hand=V(S.x+r*Math.sin(al)*Math.cos(sc), S.y-r*Math.cos(al), S.z+0.16+r*Math.abs(Math.sin(al))*Math.sin(sc));
    between(c,pul,hand); between(hd,hand.clone().add(V(0,-0.05,0)),hand.clone().add(V(0,0.05,0)));
    man.set({...STAND, hand:{L:V(-0.24,0.98,0.06),R:hand}, foot:feetStand(0.14), elbowPole:{L:V(-1,0,-0.3),R:V(0.6,0.4,-0.6)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Élévations latérales haltères"]={ cam:{target:V(0,1.0,0),dist:3.2,h:1.25,ang:15}, build(scene){
  floor(scene); const man=Man(scene); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=V(0.2*s,1.43,0), al=lerp(8,84,t)*D, r=0.575, sc=20*D;
      h[k]=V(S.x+s*r*Math.sin(al)*Math.cos(sc), S.y-r*Math.cos(al), S.z+0.06+r*Math.sin(al)*Math.sin(sc)); placeDB(dbs[i++],h[k],V(0,0,1)); }
    man.set({...STAND, hand:h, foot:feetStand(0.14), elbowPole:{L:V(-0.6,0.4,-0.6),R:V(0.6,0.4,-0.6)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Extension triceps overhead poulie"]={ cam:{target:V(0,1.05,-0.2),dist:3.4,h:1.3,ang:70}, build(scene){
  floor(scene); const man=Man(scene); const P=tower(scene,0,-0.95,1.95,0); const c=cable(scene), r1=rod(scene,0.012,MAT.rope), r2=rod(scene,0.012,MAT.rope);
  const T=tiltVecs(45), pelvis=V(0,0.86,-0.05);
  return { man, pose(t){
    const right=new THREE.Vector3().crossVectors(T.up,T.fwd).normalize(); const C=pelvis.clone().addScaledVector(T.up,0.5); const h={}, el={};
    const ext=lerp(135,5,t)*D;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().addScaledVector(right,0.2*s);
      const E=C.clone().addScaledVector(right,0.10*s).addScaledVector(T.up,0.27).addScaledVector(T.fwd,0.10); el[k]=E;
      const u=new THREE.Vector3().subVectors(E,S).normalize(); const back=T.fwd.clone().negate();
      h[k]=E.clone().addScaledVector(u,0.30*Math.cos(ext)).addScaledVector(back,0.30*Math.sin(ext)).addScaledVector(right,-0.04*s); }
    const knot=h.L.clone().add(h.R).multiplyScalar(0.5).addScaledVector(T.up,-0.10).addScaledVector(T.fwd,-0.03);
    between(c,P,knot); between(r1,knot,h.L); between(r2,knot,h.R);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, elbow:el, foot:{L:V(-0.13,0.07,0.38),R:V(0.13,0.10,-0.45)},
      elbowPole:{L:V(0,0,1),R:V(0,0,1)}, kneePole:{L:V(-0.1,0,1),R:V(0.1,-0.3,1)}, footDir:{L:V(-0.1,0,1),R:V(0.1,-0.5,1)}}); }};
}};

EX["Pushdown corde"]={ cam:{target:V(0,1.1,0.1),dist:3.3,h:1.35,ang:60}, build(scene){
  floor(scene); const man=Man(scene); const P=tower(scene,0,0.62,2.0,0); const c=cable(scene), r1=rod(scene,0.012,MAT.rope), r2=rod(scene,0.012,MAT.rope);
  const T=tiltVecs(12), pelvis=V(0,0.91,0.0);
  return { man, pose(t){ const right=new THREE.Vector3().crossVectors(T.up,T.fwd).normalize(); const C=pelvis.clone().addScaledVector(T.up,0.5); const h={}, el={};
    const ang=lerp(-10,88,t)*D; // 0 = avant-bras horizontal vers l'avant, 90 = vertical vers le bas
    for(const [k,s] of [['L',-1],['R',1]]){ const E=C.clone().addScaledVector(right,0.19*s).add(V(0,-0.28,0)).addScaledVector(T.fwd,0.03); el[k]=E;
      const spread=lerp(0.06,0.14,t); h[k]=E.clone().add(V(0,-0.30*Math.sin(ang),0)).add(new THREE.Vector3(T.fwd.x,0,T.fwd.z).normalize().multiplyScalar(0.30*Math.cos(ang))).addScaledVector(right,(spread-0.19)*s); }
    const knot=h.L.clone().add(h.R).multiplyScalar(0.5).add(V(0,0.12,0.02)); between(c,P,knot); between(r1,knot,h.L); between(r2,knot,h.R);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, elbow:el, foot:feetStand(0.15), elbowPole:{L:V(0,0,-1),R:V(0,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Pushdown unilatéral prise supination"]={ cam:{target:V(0,1.1,0.1),dist:3.3,h:1.35,ang:60}, build(scene){
  floor(scene); const man=Man(scene); const P=tower(scene,0.15,0.62,2.0,0); const c=cable(scene), hd=rod(scene,0.02,MAT.dark);
  const pelvis=V(0,0.93,0.0);
  return { man, pose(t){ const ang=lerp(0,88,t)*D; const E=V(0.19,1.15,0.03);
    const H=E.clone().add(V(0,-0.30*Math.sin(ang),0.30*Math.cos(ang)));
    between(c,P,H.clone().add(V(0,0.03,0))); between(hd,H.clone().add(V(-0.05,0,0)),H.clone().add(V(0.05,0,0)));
    man.set({...STAND, hand:{L:V(-0.24,0.98,0.06),R:H}, elbow:{R:E}, foot:feetStand(0.15), elbowPole:{L:V(-1,0,-0.3),R:V(0,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Développé épaules machine"]={ cam:{target:V(0,1.15,0),dist:3.4,h:1.4,ang:50}, build(scene){
  floor(scene); const man=Man(scene);
  const seat=box(0.36,0.07,0.36,MAT.pad); seat.position.set(0,0.47,0.05); scene.add(seat); fixed(scene,0.035,V(0,0.44,0.05),V(0,0,0.05));
  const back=box(0.36,0.75,0.07,MAT.pad); back.position.set(0,0.95,-0.17); back.rotation.x=-8*D; scene.add(back);
  for(const x of [-0.42,0.42]) fixed(scene,0.035,V(x,0,-0.32),V(x,2.0,-0.32)); fixed(scene,0.035,V(-0.42,2.0,-0.32),V(0.42,2.0,-0.32));
  const arms=[rod(scene,0.025,MAT.steel),rod(scene,0.025,MAT.steel)], hdl=[rod(scene,0.022,MAT.dark),rod(scene,0.022,MAT.dark)], car=[box(0.07,0.1,0.07,MAT.steel),box(0.07,0.1,0.07,MAT.steel)]; car.forEach(c=>scene.add(c));
  const T=tiltVecs(-8), pelvis=V(0,0.60,0.0);
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ h[k]=C.clone().add(V(s*lerp(0.34,0.24,t),lerp(0.02,0.55,t),lerp(0.14,0.06,t)));
      car[i].position.set(0.42*s,h[k].y,-0.32); between(arms[i],V(0.42*s,h[k].y,-0.32),h[k].clone().add(V(0.06*s,0,-0.02))); between(hdl[i],h[k].clone().add(V(0,0,-0.07)),h[k].clone().add(V(0,0,0.07))); i++; }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.18,0.07,0.45),R:V(0.18,0.07,0.45)}, elbowPole:{L:V(-1,-1,0.35),R:V(1,-1,0.35)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Squat barre"]={ cam:{target:V(0,0.85,0),dist:3.4,h:1.25,ang:58}, build(scene){
  floor(scene); const man=Man(scene); const bar=rod(scene,0.016,MAT.steel); const plates=[];
  for(const x of [-0.86,-0.91,0.86,0.91]){ const p=cyl(0.225,0.225,0.045,MAT.dark); p.rotation.z=Math.PI/2; p.userData.x=x; scene.add(p); plates.push(p); }
  return { man, pose(t){ const tilt=lerp(8,42,t), T=tiltVecs(tilt), a=tilt*D;
    const pelvis=V(0,lerp(0.93,0.50,t),0.03-0.5*Math.sin(a)+0.08*Math.cos(a)); const barC=pelvis.clone().addScaledVector(T.up,0.53).addScaledVector(T.fwd,-0.09);
    between(bar,barC.clone().add(V(-1.05,0,0)),barC.clone().add(V(1.05,0,0))); plates.forEach(p=>p.position.set(p.userData.x,barC.y,barC.z));
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:barC.clone().add(V(-0.43,0,0)),R:barC.clone().add(V(0.43,0,0))}, foot:{L:V(-0.17,0.07,0),R:V(0.17,0.07,0)},
      elbowPole:{L:V(-0.3,-1,-0.6),R:V(0.3,-1,-0.6)}, kneePole:{L:V(-0.35,0,1),R:V(0.35,0,1)}, footDir:{L:V(-0.27,0,1),R:V(0.27,0,1)}}); }};
}};

EX["Tirage vertical prise neutre serrée"]={ cam:{target:V(0,1.1,0.1),dist:3.6,h:1.4,ang:50}, build(scene){
  floor(scene); const man=Man(scene);
  const seat=box(0.38,0.07,0.38,MAT.pad); seat.position.set(0,0.455,0); scene.add(seat); fixed(scene,0.035,V(0,0.42,0),V(0,0,0));
  fixed(scene,0.045,V(0,0,0.78),V(0,2.32,0.78)); fixed(scene,0.04,V(0,2.32,0.78),V(0,2.32,0.08));
  const pul=cyl(0.055,0.055,0.03,MAT.steel); pul.rotation.z=Math.PI/2; pul.position.set(0,2.26,0.08); scene.add(pul);
  const tp=cyl(0.055,0.055,0.5,MAT.pad); tp.rotation.z=Math.PI/2; tp.position.set(0,0.705,0.30); scene.add(tp);
  fixed(scene,0.03,V(0,0.705,0.30),V(0,0.42,0.62)); fixed(scene,0.03,V(0,0.42,0.62),V(0,0,0.70)); const st=box(0.3,0.9,0.18,MAT.dark); st.position.set(0,0.5,0.98); scene.add(st);
  const c=cable(scene), vbar=rod(scene,0.018,MAT.dark), vL=rod(scene,0.014,MAT.dark), vR=rod(scene,0.014,MAT.dark);
  const pelvis=V(0,0.585,-0.05), T=tiltVecs(-14);
  return { man, pose(t){ const hc=V(0,lerp(1.66,1.05,t),lerp(0.03,0.16,t));
    const hL=hc.clone().add(V(-0.06,0,0)), hR=hc.clone().add(V(0.06,0,0)), top=hc.clone().add(V(0,0.12,0));
    between(c,V(0,2.24,0.08),top); between(vbar,hL.clone().add(V(0,-0.05,0)),hR.clone().add(V(0,-0.05,0))); between(vL,hL.clone().add(V(0,0.06,0)),top); between(vR,hR.clone().add(V(0,0.06,0)),top);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:hL,R:hR}, foot:{L:V(-0.13,0.07,0.43),R:V(0.13,0.07,0.43)},
      elbowPole:{L:V(-0.5,-1,0.2),R:V(0.5,-1,0.2)}, kneePole:{L:V(0,0.2,1),R:V(0,0.2,1)}}); }};
}};

// ---------------- Contrôle de collisions (outil de vérification) ----------------
function segDist(p1,q1,p2,q2){ // distance entre segments
  const d1=new THREE.Vector3().subVectors(q1,p1), d2=new THREE.Vector3().subVectors(q2,p2), r=new THREE.Vector3().subVectors(p1,p2);
  const a=d1.dot(d1), e=d2.dot(d2), f=d2.dot(r); let s,t;
  if(a<=1e-9&&e<=1e-9) return r.length();
  if(a<=1e-9){ s=0; t=Math.min(1,Math.max(0,f/e)); } else { const c=d1.dot(r); if(e<=1e-9){ t=0; s=Math.min(1,Math.max(0,-c/a)); } else { const b=d1.dot(d2), den=a*e-b*b; s=den!==0?Math.min(1,Math.max(0,(b*f-c*e)/den)):0; t=(b*s+f)/e; if(t<0){t=0;s=Math.min(1,Math.max(0,-c/a));} else if(t>1){t=1;s=Math.min(1,Math.max(0,(b-c)/a));} } }
  return p1.clone().addScaledVector(d1,s).sub(p2.clone().addScaledVector(d2,t)).length();
}
function checkCables(scene,man){ const out=[]; const caps=man.capsules();
  scene.traverse(o=>{ if(!o.userData||!o.userData.cable||!o.userData.a) return; const a=o.userData.a, b=o.userData.b; const L=a.distanceTo(b); if(L<0.15) return;
    const b2=a.clone().lerp(b,(L-0.10)/L); // on ignore les 10 derniers cm (accroche à la main)
    for(const c of caps){ const d=segDist(a,b2,c.a,c.b); if(d<c.r) out.push(c.n+' ('+(d-c.r).toFixed(3)+')'); } });
  return out; }

window.EXO3D={EX, D, V, checkCables, makeScene(){ const s=new THREE.Scene(); s.add(new THREE.HemisphereLight(0xffffff,0x5a5a5a,1.25));
  const d=new THREE.DirectionalLight(0xffffff,1.6); d.position.set(2,4,3); s.add(d); const d2=new THREE.DirectionalLight(0xffffff,0.5); d2.position.set(-3,2,-2); s.add(d2); return s; },
  setCam(cam,c,off){ const a=(c.ang+(off||0))*D; cam.position.set(c.target.x+c.dist*Math.sin(a), c.h, c.target.z+c.dist*Math.cos(a)); cam.lookAt(c.target); }};
})();
