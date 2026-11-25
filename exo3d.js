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
  const upper=new THREE.Group(); upper.position.y=0.25; torso.add(upper); // haut du buste (peut s'arrondir / se cambrer)
  const chest=sph(0.19,MAT.body); chest.position.y=0.15; chest.scale.set(1.08,1.05,0.72); upper.add(chest);
  const neck=cyl(0.05,0.055,0.12,MAT.body); neck.position.y=0.35; upper.add(neck);
  const head=sph(0.11,MAT.body); head.position.set(0,0.48,0.01); head.scale.set(0.9,1.08,1); upper.add(head);
  const parts={}; const caps=[];
  for(const k of ['L','R']){
    parts['sh'+k]=sph(0.072,MAT.body); parts['el'+k]=sph(0.048,MAT.body); parts['hd'+k]=sph(0.045,MAT.body);
    parts['ua'+k]=cyl(0.045,0.058,1,MAT.body); parts['fa'+k]=cyl(0.034,0.045,1,MAT.body);
    parts['hp'+k]=sph(0.085,MAT.body); parts['kn'+k]=sph(0.06,MAT.body);
    parts['th'+k]=cyl(0.06,0.085,1,MAT.body); parts['sn'+k]=cyl(0.042,0.058,1,MAT.body);
    parts['ft'+k]=box(0.09,0.06,0.22,MAT.body);
    parts['pm'+k]=box(0.09,0.03,0.12,MAT.body); parts['pm'+k].visible=false;
    for(const n of ['sh','el','hd','ua','fa','hp','kn','th','sn','ft','pm']) scene.add(parts[n+k]);
  }
  const J={};
  function set(P){
    const up=P.up.clone().normalize(), fwd=P.fwd.clone().normalize(), right=new THREE.Vector3().crossVectors(up,fwd).normalize();
    const f2=new THREE.Vector3().crossVectors(right,up).normalize();
    torso.position.copy(P.pelvis); torso.quaternion.copy(basisQuat(right,up,f2)); upper.rotation.set(P.bend||0,P.twist||0,0); torso.updateMatrixWorld(true);
    J.up=up; J.fwd=f2; J.right=right; J.pelvis=P.pelvis.clone();
    const uq=upper.getWorldQuaternion(new THREE.Quaternion()); J.cn=V(0,0,1).applyQuaternion(uq); J.cu=V(0,1,0).applyQuaternion(uq);
    J.neck=upper.localToWorld(V(0,0.30,0)); J.head=upper.localToWorld(V(0,0.48,0.01)); J.chest=upper.localToWorld(V(0,0.13,0)); J.abd=torso.localToWorld(V(0,0.12,0));
    for(const [k,s] of [['L',-1],['R',1]]){
      const S=upper.localToWorld(V(0.2*s,0.25+(P.shrug||0),0)), H=P.hand[k];
      const E=P.elbow&&P.elbow[k] ? P.elbow[k] : ik3(S,H,BODY.ua,BODY.fa,P.elbowPole[k]);
      parts['sh'+k].position.copy(S); parts['el'+k].position.copy(E); parts['hd'+k].position.copy(H);
      between(parts['ua'+k],S,E); between(parts['fa'+k],E,H);
      const pm=P.palm&&P.palm[k]; parts['pm'+k].visible=!!pm; parts['hd'+k].visible=!pm;
      if(pm){ const d=pm.dir.clone().normalize(), n=pm.normal.clone().addScaledVector(d,-pm.normal.dot(d)).normalize(), r=new THREE.Vector3().crossVectors(n,d).normalize();
        parts['pm'+k].quaternion.copy(basisQuat(r,n,d)); parts['pm'+k].position.copy(H).addScaledVector(d,0.04); }
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

EX["Développé incliné haltères"]={ cam:{target:V(0,0.95,0),dist:3.1,h:1.45,ang:62}, build(scene){
  floor(scene); const man=Man(scene); const inc=35, hinge=V(0,0.44,0.25); bench(scene,inc,hinge); const F=benchFrame(inc,hinge);
  const C=F.pelvis.clone().addScaledVector(F.up,0.5), dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const h={};
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().add(V(0.2*s,0,0));
      // départ : haltères au niveau du haut des pectoraux ; arrivée : bras verticaux au-dessus des épaules (trajet vers le plafond, pas dans l'axe du banc)
      const A=S.clone().add(V(0.14*s,0,0)).addScaledVector(F.fwd,0.13).addScaledVector(F.up,-0.10), B=S.clone().add(V(-0.07*s,0.565,0.03));
      const e=ease(t); h[k]=A.clone().lerp(B,e); h[k].x=lerp(A.x,B.x,Math.min(1,e*1.15)); }
    placeDB(dbs[0],h.L,V(1,0,0)); placeDB(dbs[1],h.R,V(1,0,0));
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:h, foot:{L:V(-0.26,0.07,F.pelvis.z+0.50),R:V(0.26,0.07,F.pelvis.z+0.50)},
      elbowPole:{L:V(-1,0,0).addScaledVector(F.fwd,-1.0).addScaledVector(F.up,-0.3),R:V(1,0,0).addScaledVector(F.fwd,-1.0).addScaledVector(F.up,-0.3)},
      kneePole:{L:V(-0.2,1,0.2),R:V(0.2,1,0.2)}, footDir:toesOut}); }};
}};

// Dips : épaules qui passent devant les mains en bas, coudes vers l'arrière, buste penché ~25°, genoux fléchis pieds derrière

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

// =====================================================================
//  Lot 2 : alternatives Push, Pull, Legs, abdos
// =====================================================================
function barbell(scene,len,pr,n){ const bar=rod(scene,0.016,MAT.steel); const pl=[];
  for(let i=0;i<(n||1);i++) for(const s of [-1,1]){ const p=cyl(pr,pr,0.04,MAT.dark); p.userData.o=s*(len/2-0.15-i*0.045); scene.add(p); pl.push(p); }
  return function(c,axis){ axis=(axis||V(1,0,0)).clone().normalize(); between(bar,c.clone().addScaledVector(axis,-len/2),c.clone().addScaledVector(axis,len/2));
    pl.forEach(p=>{ p.position.copy(c).addScaledVector(axis,p.userData.o); p.quaternion.setFromUnitVectors(V(0,1,0),axis); }); }; }
function ezbar(scene){ const set=barbell(scene,1.25,0.12,1); return set; }
function frameOf(up,fwd){ const r=new THREE.Vector3().crossVectors(up,fwd).normalize(); return r; }
// banc incliné "ventre contre le dossier" (le dossier monte vers +z)
function proneBench(scene,incl,base){ const a=incl*D, dir=V(0,Math.sin(a),Math.cos(a)), n=V(0,Math.cos(a),-Math.sin(a));
  const g=new THREE.Group(); g.position.copy(base); g.quaternion.setFromUnitVectors(V(0,0,1),dir); scene.add(g);
  const pad=box(0.32,0.07,0.85,MAT.pad); pad.position.set(0,0,0.42); g.add(pad);
  const seat=box(0.30,0.06,0.25,MAT.pad); seat.position.copy(base).add(V(0,-0.12,-0.12)); scene.add(seat);
  const top=base.clone().addScaledVector(dir,0.75); fixed(scene,0.03,top.clone().addScaledVector(n,-0.04),V(top.x,0,top.z)); fixed(scene,0.035,base.clone().add(V(0,-0.15,-0.1)),V(base.x,0,base.z-0.1));
  const f=box(0.42,0.04,1.2,MAT.steel); f.position.set(base.x,0.02,base.z+0.3); scene.add(f);
  return {dir,n}; }
function seatBox(scene,y,z,w){ const s=box(w||0.38,0.07,0.38,MAT.pad); s.position.set(0,y,z); scene.add(s); fixed(scene,0.035,V(0,y-0.03,z),V(0,0,z)); return s; }
function backPad(scene,center,angDeg,h){ const b=box(0.36,h||0.7,0.07,MAT.pad); b.position.copy(center); b.rotation.x=-angDeg*D; scene.add(b); return b; }
const ARMS_SIDE={L:V(-0.25,0.86,0.02),R:V(0.25,0.86,0.02)};
const legsSeated=(pz)=>({L:V(-0.18,0.07,pz),R:V(0.18,0.07,pz)});

// ---------- PUSH : alternatives ----------
EX["Développé couché haltères"]={ cam:{target:V(0,0.7,0),dist:3.0,h:1.45,ang:50}, build(scene){
  floor(scene); const man=Man(scene); const hinge=V(0,0.44,0.40); bench(scene,0,hinge); const F=benchFrame(0,hinge);
  const C=F.pelvis.clone().addScaledVector(F.up,0.5), dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const h={};
    for(const [k,s] of [['L',-1],['R',1]]) h[k]=C.clone().add(V(s*lerp(0.33,0.12,t),0,0)).addScaledVector(F.fwd,lerp(0.15,0.575,t)).addScaledVector(F.up,lerp(-0.12,-0.06,t));
    placeDB(dbs[0],h.L,V(1,0,0)); placeDB(dbs[1],h.R,V(1,0,0));
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:h, foot:{L:V(-0.26,0.07,F.pelvis.z+0.50),R:V(0.26,0.07,F.pelvis.z+0.50)},
      elbowPole:{L:V(-1,-1,0.2),R:V(1,-1,0.2)}, kneePole:{L:V(-0.2,1,0.2),R:V(0.2,1,0.2)}, footDir:toesOut}); }};
}};

EX["Développé incliné machine"]={ cam:{target:V(0,1.05,0.1),dist:3.4,h:1.5,ang:55}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.47,0.02); const T=tiltVecs(-28), pelvis=V(0,0.60,-0.02);
  backPad(scene,pelvis.clone().addScaledVector(T.up,0.4).addScaledVector(T.fwd,-0.17),28,0.75);
  for(const x of [-0.45,0.45]){ fixed(scene,0.04,V(x,0,-0.45),V(x,1.95,-0.45)); }
  const piv={L:V(-0.45,1.85,-0.45),R:V(0.45,1.85,-0.45)}; const arms=[rod(scene,0.03,MAT.steel),rod(scene,0.03,MAT.steel)], gr=[rod(scene,0.022,MAT.dark),rod(scene,0.022,MAT.dark)];
  return { man, pose(t){ const right=frameOf(T.up,T.fwd), C=pelvis.clone().addScaledVector(T.up,0.5); const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ h[k]=C.clone().addScaledVector(right,s*lerp(0.33,0.17,t)).addScaledVector(T.fwd,lerp(0.14,0.50,t)).addScaledVector(T.up,lerp(-0.10,0.26,t));
      between(arms[i],piv[k],h[k].clone().add(V(0.07*s,0,0))); between(gr[i],h[k].clone().add(V(-0.06,0,0)),h[k].clone().add(V(0.06,0,0))); i++; }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:legsSeated(0.48), elbowPole:{L:V(-1,-1,0),R:V(1,-1,0)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Pec deck"]={ cam:{target:V(0,1.1,0.05),dist:3.4,h:1.5,ang:30}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.50,0.0); const T=tiltVecs(-4), pelvis=V(0,0.63,-0.02);
  backPad(scene,pelvis.clone().addScaledVector(T.up,0.42).add(V(0,0,-0.17)),4,0.8); fixed(scene,0.04,V(0,0,-0.30),V(0,2.0,-0.30)); fixed(scene,0.035,V(-0.3,2.0,-0.30),V(0.3,2.0,-0.30));
  const arms=[rod(scene,0.03,MAT.steel),rod(scene,0.03,MAT.steel)], hd=[rod(scene,0.022,MAT.dark),rod(scene,0.022,MAT.dark)];
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().add(V(0.2*s,0,0)); const d=slerpDir(V(s*0.98,-0.05,0.15),V(-s*0.2,-0.05,0.98),t); h[k]=S.clone().addScaledVector(d,0.56);
      const piv=V(0.25*s,2.0,-0.30); between(arms[i],piv,h[k].clone().add(V(0,0.12,0))); between(hd[i],h[k].clone().add(V(0,-0.07,0)),h[k].clone().add(V(0,0.12,0))); i++; }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:legsSeated(0.45), elbowPole:{L:V(-0.4,-0.3,-1),R:V(0.4,-0.3,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Écartés haltères banc incliné"]={ cam:{target:V(0,0.8,0),dist:3.1,h:1.5,ang:40}, build(scene){
  floor(scene); const man=Man(scene); const inc=30, hinge=V(0,0.44,0.25); bench(scene,inc,hinge); const F=benchFrame(inc,hinge);
  const C=F.pelvis.clone().addScaledVector(F.up,0.5), right=frameOf(F.up,F.fwd), dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().addScaledVector(right,0.2*s);
      const a=right.clone().multiplyScalar(s*0.96).addScaledVector(F.fwd,0.08).addScaledVector(F.up,-0.22), b=right.clone().multiplyScalar(-s*0.16).addScaledVector(F.fwd,0.99);
      h[k]=S.clone().addScaledVector(slerpDir(a,b,t),0.55); placeDB(dbs[i++],h[k],F.up); }
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:h, foot:{L:V(-0.26,0.07,F.pelvis.z+0.50),R:V(0.26,0.07,F.pelvis.z+0.50)},
      elbowPole:{L:right.clone().negate().addScaledVector(F.fwd,-0.6),R:right.clone().addScaledVector(F.fwd,-0.6)}, kneePole:{L:V(-0.2,1,0.2),R:V(0.2,1,0.2)}, footDir:toesOut}); }};
}};

EX["Développé épaules haltères assis"]={ cam:{target:V(0,1.15,0),dist:3.2,h:1.45,ang:35}, build(scene){
  floor(scene); const man=Man(scene); const inc=82, hinge=V(0,0.44,0.15); bench(scene,inc,hinge); const F=benchFrame(inc,hinge);
  const C=F.pelvis.clone().addScaledVector(F.up,0.5), dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ h[k]=C.clone().add(V(s*lerp(0.36,0.13,t),0,0)).addScaledVector(F.up,lerp(0.06,0.56,t)).addScaledVector(F.fwd,lerp(0.10,0.04,t)); placeDB(dbs[i++],h[k],V(1,0,0)); }
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:h, foot:legsSeated(F.pelvis.z+0.48), elbowPole:{L:V(-1,-1,0.15),R:V(1,-1,0.15)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Élévations latérales machine"]={ cam:{target:V(0,1.1,0),dist:3.3,h:1.5,ang:155}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.50,0.02); const pelvis=V(0,0.63,0), up=V(0,1,0), fwd=V(0,0,1);
  const chestPad=box(0.32,0.45,0.07,MAT.pad); chestPad.position.set(0,1.05,0.22); scene.add(chestPad); fixed(scene,0.035,V(0,0.83,0.27),V(0,0,0.27));
  const pads=[cyl(0.05,0.05,0.16,MAT.pad),cyl(0.05,0.05,0.16,MAT.pad)]; pads.forEach(p=>scene.add(p)); const lev=[rod(scene,0.025,MAT.steel),rod(scene,0.025,MAT.steel)];
  return { man, pose(t){ const h={}, el={}; let i=0; const C=V(0,1.13,0);
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().add(V(0.2*s,0,0)); const ab=lerp(10,85,t)*D;
      el[k]=S.clone().add(V(s*0.29*Math.sin(ab),-0.29*Math.cos(ab),0.04)); h[k]=el[k].clone().add(V(0,0,0.28)).add(V(-0.05*s,0.02,0));
      const pd=el[k].clone().add(V(0.06*s,0.05*Math.sin(ab),0)).addScaledVector(S.clone().sub(el[k]).normalize(),-0.02); pads[i].position.copy(el[k].clone().add(V(s*0.06*Math.cos(ab),0.06*Math.sin(ab),-0.02)));
      pads[i].quaternion.setFromUnitVectors(V(0,1,0),S.clone().sub(el[k]).normalize()); between(lev[i],S.clone().add(V(0,0,-0.12)),pads[i].position.clone().add(V(0,0,-0.06))); i++; }
    man.set({pelvis, up, fwd, hand:h, elbow:el, foot:legsSeated(0.45), elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

function overheadDB(scene,oneArm){ floor(scene); const man=Man(scene);
  seatBox(scene,0.48,0.0,0.40); const pelvis=V(0,0.62,0.0), up=V(0,1,0), fwd=V(0,0,1), right=V(1,0,0);
  backPad(scene,V(0,0.86,-0.17),0,0.42); fixed(scene,0.035,V(0,0.66,-0.21),V(0,0.44,-0.10));
  const db=dumbbell(scene); const C=pelvis.clone().addScaledVector(up,0.5);
  return { man, pose(t){ const ext=lerp(125,5,ease(t))*D, h={}, el={}; const u=V(0,1,0.18).normalize(), w=V(0,0.18,-1).normalize();
    if(!oneArm){
      for(const [k,s] of [['L',-1],['R',1]]){ el[k]=C.clone().add(V(0.125*s,0.26,0.08));
        h[k]=el[k].clone().addScaledVector(u,0.30*Math.cos(ext)).addScaledVector(w,0.30*Math.sin(ext)); h[k].x=0.045*s; }
      const mid=h.L.clone().add(h.R).multiplyScalar(0.5); placeDB(db,mid.clone().add(V(0,-0.085,0)),V(0,1,0)); // tenu sous le disque du haut, l'haltère pend à la verticale
      man.set({pelvis, up, fwd, hand:h, elbow:el, foot:legsSeated(0.46), elbowPole:{L:V(0,0,1),R:V(0,0,1)}, kneePole:kneeFwd, footDir:toesOut});
    } else {
      el.R=C.clone().add(V(0.12,0.26,0.07)); const dir=V(-0.45,0,-1).normalize(); // l'avant-bras descend derrière la tête, vers l'épaule opposée
      h.R=el.R.clone().addScaledVector(u,0.30*Math.cos(ext)).addScaledVector(dir,0.30*Math.sin(ext)); const fd=h.R.clone().sub(el.R).normalize();
      const ax=V(0,0,1).addScaledVector(fd,-fd.z).normalize(); placeDB(db,h.R,ax);
      h.L=el.R.clone().add(V(-0.04,-0.09,0.06));
      man.set({pelvis, up, fwd, hand:h, elbow:{R:el.R}, foot:legsSeated(0.46), elbowPole:{L:V(-1,-0.5,0.8),R:V(0,0,1)}, kneePole:kneeFwd, footDir:toesOut}); } }};
}
EX["Extension nuque haltère deux mains"]={ cam:{target:V(0,1.2,0),dist:3.3,h:1.55,ang:125}, build(scene){ return overheadDB(scene,false); }};
EX["Extension nuque unilatérale haltère"]={ cam:{target:V(0,1.2,0),dist:3.3,h:1.55,ang:125}, build(scene){ return overheadDB(scene,true); }};

EX["Dips machine assis"]={ cam:{target:V(0,0.95,0),dist:3.2,h:1.35,ang:55}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.50,0.02); const T=tiltVecs(-6), pelvis=V(0,0.63,0);
  backPad(scene,pelvis.clone().addScaledVector(T.up,0.4).add(V(0,0,-0.17)),6,0.75);
  const lev=[rod(scene,0.03,MAT.steel),rod(scene,0.03,MAT.steel)], gr=[rod(scene,0.022,MAT.dark),rod(scene,0.022,MAT.dark)];
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ h[k]=C.clone().add(V(s*0.30,lerp(-0.26,-0.56,t),lerp(0.08,0.05,t)));
      between(lev[i],V(0.42*s,1.05,-0.35),h[k].clone().add(V(0.06*s,0,-0.06))); between(gr[i],h[k].clone().add(V(0,0,-0.07)),h[k].clone().add(V(0,0,0.07))); i++; }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:legsSeated(0.45), elbowPole:{L:V(-0.2,0,-1),R:V(0.2,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

// appui genou + main sur banc plat (kickback, rowing unilatéral)
function benchSupport(scene){ const b=box(0.30,0.07,1.2,MAT.pad); b.position.set(-0.16,0.46,0.0); scene.add(b);
  for(const z of [-0.45,0.45]) fixed(scene,0.035,V(-0.16,0.42,z),V(-0.16,0,z)); const f=box(0.4,0.04,1.2,MAT.steel); f.position.set(-0.16,0.02,0); scene.add(f);
  const up=V(0,0.12,1).normalize(), fwd=V(0,-1,0.12).normalize(); const pelvis=V(-0.02,0.78,-0.28);
  return {up,fwd,pelvis, foot:{L:V(-0.14,0.55,-0.72),R:V(0.28,0.07,-0.30)}, footDir:{L:V(0,-0.15,-1),R:V(0.1,0,1)}, kneePole:{L:V(0,-1,0.4),R:V(0.3,0,1)}, handL:V(-0.18,0.50,0.30)}; }
EX["Kickback haltère"]={ cam:{target:V(0,0.75,0),dist:3.1,h:1.3,ang:80}, build(scene){
  floor(scene); const man=Man(scene); const B=benchSupport(scene); const db=dumbbell(scene);
  return { man, pose(t){ const right=frameOf(B.up,B.fwd); const C=B.pelvis.clone().addScaledVector(B.up,0.5); const S=C.clone().addScaledVector(right,0.2);
    const E=S.clone().addScaledVector(B.up,-0.27).addScaledVector(B.fwd,-0.10); const ang=lerp(90,0,t)*D; // 90 = avant-bras vertical, 0 = tendu vers l'arrière
    const H=E.clone().add(V(0,-0.30*Math.sin(ang),0)).add(V(0,0,-0.30*Math.cos(ang))); placeDB(db,H,V(0,-Math.cos(ang),Math.sin(ang)));
    man.set({pelvis:B.pelvis, up:B.up, fwd:B.fwd, hand:{L:B.handL,R:H}, elbow:{R:E}, foot:B.foot, elbowPole:{L:V(-0.5,0,-1),R:V(0,1,0)}, kneePole:B.kneePole, footDir:B.footDir}); }};
}};
EX["Rowing haltère unilatéral"]={ cam:{target:V(0,0.75,0),dist:3.1,h:1.3,ang:80}, build(scene){
  floor(scene); const man=Man(scene); const B=benchSupport(scene); const db=dumbbell(scene);
  return { man, pose(t){ const right=frameOf(B.up,B.fwd); const C=B.pelvis.clone().addScaledVector(B.up,0.5); const S=C.clone().addScaledVector(right,0.2);
    const H=S.clone().add(V(0.02,-0.56,0.02)).lerp(B.pelvis.clone().addScaledVector(right,0.24).add(V(0,0.02,0.05)),t); placeDB(db,H,V(0,0,1));
    man.set({pelvis:B.pelvis, up:B.up, fwd:B.fwd, hand:{L:B.handL,R:H}, foot:B.foot, elbowPole:{L:V(-0.5,0,-1),R:V(0.25,0.6,-1)}, kneePole:B.kneePole, footDir:B.footDir}); }};
}};

function dipStation(scene,weighted){ floor(scene); const man=Man(scene); const by=1.27;
  for(const x of [-0.27,0.27]){ fixed(scene,0.025,V(x,by-0.02,-0.40),V(x,by-0.02,0.40)); fixed(scene,0.035,V(x,0,-0.40),V(x,by-0.02,-0.40)); fixed(scene,0.035,V(x,0,0.40),V(x,by-0.02,0.40)); }
  let chain=null, plate=null, belt=null; if(weighted){ chain=rod(scene,0.008,MAT.steel); plate=cyl(0.16,0.16,0.04,MAT.dark); scene.add(plate); belt=cyl(0.16,0.16,0.05,MAT.dark); belt.scale.set(1.05,1,0.82); scene.add(belt); }
  return { man, pose(t){ const e=ease(t); const tilt=lerp(6,26,e), T=tiltVecs(tilt); const hand={L:V(-0.27,by,0),R:V(0.27,by,0)};
    const C=V(0,lerp(by+0.565,by+0.33,e),lerp(-0.02,0.17,e)); const pelvis=C.clone().addScaledVector(T.up,-0.5);
    const A={L:pelvis.clone().add(V(-0.06,-0.64,-0.30)),R:pelvis.clone().add(V(0.02,-0.66,-0.36))};
    if(weighted){ belt.position.copy(pelvis.clone().addScaledVector(T.up,0.02)); belt.quaternion.setFromUnitVectors(V(0,1,0),T.up);
      const w=pelvis.clone().addScaledVector(T.fwd,0.11); const pc=V(0,pelvis.y-0.60,pelvis.z+0.17); between(chain,w,pc.clone().add(V(0,0.16,0))); plate.position.copy(pc); plate.rotation.set(0,0,Math.PI/2); }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand, foot:A, elbowPole:{L:V(-0.25,0.1,-1),R:V(0.25,0.1,-1)}, kneePole:{L:V(0,-0.5,1),R:V(0,-0.5,1)}, footDir:{L:V(0,-0.8,-0.5),R:V(0,-0.8,-0.5)}}); }};
}
EX["Dips lestés"]={ cam:{target:V(0,1.15,0),dist:3.8,h:1.45,ang:72}, build(scene){ return dipStation(scene,true); }};
EX["Dips (poids du corps)"]={ cam:{target:V(0,1.15,0),dist:3.8,h:1.45,ang:72}, build(scene){ return dipStation(scene,false); }};

// Extensions nuque : assis buste droit, dossier court (s'arrête sous les épaules) pour que l'haltère passe derrière la tête

// ---------- PULL ----------
function pulldownBase(scene){ floor(scene); const man=Man(scene); seatBox(scene,0.455,0);
  fixed(scene,0.045,V(0,0,0.78),V(0,2.32,0.78)); fixed(scene,0.04,V(0,2.32,0.78),V(0,2.32,0.08));
  const pul=cyl(0.055,0.055,0.03,MAT.steel); pul.rotation.z=Math.PI/2; pul.position.set(0,2.26,0.08); scene.add(pul);
  const tp=cyl(0.055,0.055,0.5,MAT.pad); tp.rotation.z=Math.PI/2; tp.position.set(0,0.705,0.30); scene.add(tp);
  fixed(scene,0.03,V(0,0.705,0.30),V(0,0.42,0.62)); fixed(scene,0.03,V(0,0.42,0.62),V(0,0,0.70)); const st=box(0.3,0.9,0.18,MAT.dark); st.position.set(0,0.5,0.98); scene.add(st);
  return man; }
EX["Tirage vertical prise large"]={ cam:{target:V(0,1.1,0.1),dist:3.6,h:1.4,ang:30}, build(scene){
  const man=pulldownBase(scene); const c=cable(scene), bar=rod(scene,0.016,MAT.dark), bL=rod(scene,0.016,MAT.dark), bR=rod(scene,0.016,MAT.dark);
  const pelvis=V(0,0.585,-0.05), T=tiltVecs(-14);
  return { man, pose(t){ const y=lerp(1.70,1.08,t), z=lerp(0.02,0.16,t); const hL=V(-0.40,y,z), hR=V(0.40,y,z), mid=V(0,y+0.04,z);
    between(c,V(0,2.24,0.08),mid); between(bar,V(-0.30,y,z),V(0.30,y,z)); between(bL,V(-0.30,y,z),V(-0.55,y-0.06,z)); between(bR,V(0.30,y,z),V(0.55,y-0.06,z));
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:hL,R:hR}, foot:{L:V(-0.13,0.07,0.43),R:V(0.13,0.07,0.43)}, elbowPole:{L:V(-1,-1,-0.2),R:V(1,-1,-0.2)}, kneePole:{L:V(0,0.2,1),R:V(0,0.2,1)}}); }};
}};

EX["Tractions (poids du corps)"]={ cam:{target:V(0,1.45,0),dist:4.2,h:1.6,ang:35}, build(scene){
  floor(scene); const man=Man(scene); const by=2.25; fixed(scene,0.018,V(-0.7,by,0),V(0.7,by,0)); for(const x of [-0.7,0.7]) fixed(scene,0.04,V(x,0,0),V(x,by+0.1,0));
  return { man, pose(t){ const T=tiltVecs(-10), C=V(0,lerp(by-0.555,by-0.17,t),lerp(-0.04,-0.10,t)), pelvis=C.clone().addScaledVector(T.up,-0.5);
    const A={L:pelvis.clone().add(V(-0.06,-0.62,-0.34)),R:pelvis.clone().add(V(0.06,-0.60,-0.38))};
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.38,by,0),R:V(0.38,by,0)}, foot:A, elbowPole:{L:V(-1,-1,0),R:V(1,-1,0)}, kneePole:{L:V(0,0,1),R:V(0,0,1)}, footDir:{L:V(0,-0.7,-0.6),R:V(0,-0.7,-0.6)}}); }};
}};

EX["Tirage horizontal coudes serrés"]={ cam:{target:V(0,0.8,0.4),dist:4.0,h:1.45,ang:68}, build(scene){
  floor(scene); const man=Man(scene);
  const seat=box(0.36,0.08,0.95,MAT.pad); seat.position.set(0,0.46,-0.05); scene.add(seat);
  const rail=box(0.12,0.06,1.85,MAT.steel); rail.position.set(0,0.03,0.33); scene.add(rail);
  fixed(scene,0.04,V(0,0.42,-0.40),V(0,0.03,-0.40)); fixed(scene,0.04,V(0,0.42,0.30),V(0,0.03,0.30));
  // repose-pieds inclinés
  for(const x of [-0.15,0.15]){ const fp=box(0.13,0.30,0.04,MAT.steel); fp.position.set(x,0.30,0.72); fp.rotation.x=-18*D; scene.add(fp); fixed(scene,0.025,V(x,0.18,0.75),V(x,0.03,0.80)); }
  fixed(scene,0.025,V(-0.15,0.03,0.80),V(0.15,0.03,0.80));
  // tour à charges
  const zt=1.22; const base=box(0.72,0.05,0.42,MAT.steel); base.position.set(0,0.025,zt); scene.add(base);
  for(const x of [-0.27,0.27]) fixed(scene,0.04,V(x,0,zt),V(x,2.08,zt)); fixed(scene,0.04,V(-0.27,2.08,zt),V(0.27,2.08,zt));
  for(const x of [-0.08,0.08]) fixed(scene,0.008,V(x,0.05,zt),V(x,1.85,zt),MAT.steel);
  const st=box(0.30,0.55,0.14,MAT.dark); st.position.set(0,0.33,zt); scene.add(st); const mv=box(0.30,0.20,0.14,MAT.dark); scene.add(mv);
  const pul=[V(0,0.40,0.95),V(0,0.40,zt+0.13),V(0,1.98,zt+0.13),V(0,1.98,zt)];
  for(const p of pul){ const w=cyl(0.045,0.045,0.03,MAT.steel); w.rotation.z=Math.PI/2; w.position.copy(p); scene.add(w); }
  fixed(scene,0.03,V(0,0.03,0.95),V(0,0.36,0.95));
  const c=cable(scene), c2=cable(scene), c3=cable(scene), c4=cable(scene), vb=rod(scene,0.018,MAT.dark), vL=rod(scene,0.014,MAT.dark), vR=rod(scene,0.014,MAT.dark);
  between(c2,pul[0],pul[1]); between(c3,pul[1],pul[2]); between(c4,pul[2],pul[3]);
  const c5=cable(scene);
  return { man, pose(t){ const T=tiltVecs(lerp(12,-6,t)), pelvis=V(0,0.61,-0.10); const C=pelvis.clone().addScaledVector(T.up,0.5);
    const hc=V(0,lerp(C.y-0.10,0.92,t),lerp(C.z+0.56,pelvis.z+0.24,t)); const hL=hc.clone().add(V(-0.06,0,0)), hR=hc.clone().add(V(0.06,0,0)), top=hc.clone().add(V(0,0,0.11));
    between(c,pul[0],top); between(vb,hL,hR); between(vL,hL,top); between(vR,hR,top);
    const y0=0.605+0.30*t; mv.position.set(0,y0+0.10,zt); between(c5,pul[3],V(0,y0+0.20,zt));
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:hL,R:hR}, foot:{L:V(-0.15,0.30,0.64),R:V(0.15,0.30,0.64)}, elbowPole:{L:V(-0.25,-0.3,-1),R:V(0.25,-0.3,-1)}, kneePole:{L:V(-0.1,1,0.2),R:V(0.1,1,0.2)}, footDir:{L:V(0,0.3,1),R:V(0,0.3,1)}}); }};
}};

EX["Pull-over poulie"]={ cam:{target:V(0,1.1,0.1),dist:3.6,h:1.4,ang:72}, build(scene){
  floor(scene); const man=Man(scene); const P=tower(scene,0,0.95,2.05,0); const c=cable(scene), r1=rod(scene,0.012,MAT.rope), r2=rod(scene,0.012,MAT.rope);
  const T=tiltVecs(45), pelvis=V(0,0.84,-0.25);
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const d=slerpDir(V(0,0.62,0.78),V(0,-0.93,0.36),t); const h={};
    for(const [k,s] of [['L',-1],['R',1]]) h[k]=C.clone().add(V(0.2*s,0,0)).addScaledVector(d,0.56).add(V(-0.15*s,0,0));
    const knot=h.L.clone().add(h.R).multiplyScalar(0.5).addScaledVector(P.clone().sub(h.L).normalize(),0.10); between(c,P,knot); between(r1,knot,h.L); between(r2,knot,h.R);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:feetStand(0.14,0.05), elbowPole:{L:V(-0.5,0,-1),R:V(0.5,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Pull-over haltère"]={ cam:{target:V(0,0.75,0),dist:3.1,h:1.4,ang:70}, build(scene){
  floor(scene); const man=Man(scene); const hinge=V(0,0.44,0.40); bench(scene,0,hinge); const F=benchFrame(0,hinge); const db=dumbbell(scene);
  return { man, pose(t){ const C=F.pelvis.clone().addScaledVector(F.up,0.5); const d=slerpDir(F.up.clone().multiplyScalar(0.85).addScaledVector(F.fwd,-0.30),F.fwd.clone().addScaledVector(F.up,0.10),t); const h={};
    for(const [k,s] of [['L',-1],['R',1]]) h[k]=C.clone().add(V(0.2*s,0,0)).addScaledVector(d,0.55).add(V(-0.17*s,0,0));
    placeDB(db,h.L.clone().add(h.R).multiplyScalar(0.5).addScaledVector(d,0.06),d);
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:h, foot:{L:V(-0.26,0.07,F.pelvis.z+0.50),R:V(0.26,0.07,F.pelvis.z+0.50)}, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(-0.2,1,0.2),R:V(0.2,1,0.2)}, footDir:toesOut}); }};
}};

EX["Face pull"]={ cam:{target:V(0,1.3,0.2),dist:3.6,h:1.55,ang:40}, build(scene){
  floor(scene); const man=Man(scene); const P=tower(scene,0,1.15,1.72,0); const c=cable(scene), r1=rod(scene,0.012,MAT.rope), r2=rod(scene,0.012,MAT.rope);
  const pelvis=V(0,0.92,0);
  return { man, pose(t){ const h={L:V(lerp(-0.05,-0.20,t),lerp(1.48,1.58,t),lerp(0.58,0.12,t)),R:V(lerp(0.05,0.20,t),lerp(1.48,1.58,t),lerp(0.58,0.12,t))};
    const knot=V(0,lerp(1.50,1.60,t),lerp(0.70,0.36,t)); between(c,P,knot); between(r1,knot,h.L); between(r2,knot,h.R);
    man.set({...STAND, hand:h, foot:{L:V(-0.13,0.07,0.12),R:V(0.13,0.07,-0.12)}, elbowPole:{L:V(-1,0.7,-0.2),R:V(1,0.7,-0.2)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

function proneSetup(scene,incl){ floor(scene); const man=Man(scene); const base=V(0,0.55,-0.10); const B=proneBench(scene,incl,base);
  const up=B.dir.clone(), fwd=B.n.clone().negate(); const pelvis=base.clone().addScaledVector(up,0.06).addScaledVector(fwd,-0.17);
  return {man,up,fwd,pelvis, foot:{L:V(-0.14,0.07,pelvis.z-0.42),R:V(0.14,0.07,pelvis.z-0.42)}}; }
EX["Oiseau haltères buste appuyé"]={ cam:{target:V(0,0.9,0.2),dist:3.3,h:1.6,ang:25}, build(scene){
  const P=proneSetup(scene,38); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man:P.man, pose(t){ const right=frameOf(P.up,P.fwd), C=P.pelvis.clone().addScaledVector(P.up,0.5); const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().addScaledVector(right,0.2*s); const d=slerpDir(V(0,-1,0.05),V(s*0.97,0.10,0.05),t); h[k]=S.clone().addScaledVector(d,0.55); placeDB(dbs[i++],h[k],V(0,0,1)); }
    P.man.set({pelvis:P.pelvis, up:P.up, fwd:P.fwd, hand:h, foot:P.foot, elbowPole:{L:V(-0.3,1,-0.3),R:V(0.3,1,-0.3)}, kneePole:{L:V(0,0,1),R:V(0,0,1)}}); }};
}};
EX["Rowing haltères buste appuyé"]={ cam:{target:V(0,0.9,0.2),dist:3.3,h:1.5,ang:70}, build(scene){
  const P=proneSetup(scene,38); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man:P.man, pose(t){ const right=frameOf(P.up,P.fwd), C=P.pelvis.clone().addScaledVector(P.up,0.5); const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().addScaledVector(right,0.2*s); h[k]=S.clone().add(V(0.02*s,-0.56,0.03)).lerp(S.clone().addScaledVector(P.up,-0.30).addScaledVector(P.fwd,0.08).addScaledVector(right,0.05*s),t); placeDB(dbs[i++],h[k],V(1,0,0)); }
    P.man.set({pelvis:P.pelvis, up:P.up, fwd:P.fwd, hand:h, foot:P.foot, elbowPole:{L:V(-0.3,1,-0.6),R:V(0.3,1,-0.6)}, kneePole:{L:V(0,0,1),R:V(0,0,1)}}); }};
}};

EX["Rowing machine assis poitrine appuyée"]={ cam:{target:V(0,0.8,0.15),dist:3.7,h:1.5,ang:-62}, build(scene){
  floor(scene); const man=Man(scene);
  // châssis au sol
  for(const x of [-0.58,0.58]){ const r=box(0.07,0.06,1.55,MAT.steel); r.position.set(x,0.03,0.22); scene.add(r); }
  for(const z of [-0.52,0.96]){ const r=box(1.23,0.06,0.07,MAT.steel); r.position.set(0,0.03,z); scene.add(r); }
  const spine=box(0.08,0.06,1.48,MAT.steel); spine.position.set(0,0.03,0.22); scene.add(spine);
  // siège
  const seat=box(0.36,0.08,0.36,MAT.pad); seat.position.set(0,0.50,-0.14); scene.add(seat); fixed(scene,0.04,V(0,0.46,-0.14),V(0,0.03,-0.14)); fixed(scene,0.03,V(0,0.30,-0.14),V(0,0.03,-0.42));
  // coussin poitrine sur son montant
  const T=tiltVecs(8), pelvis=V(0,0.65,-0.14);
  const pad=box(0.30,0.40,0.07,MAT.pad); pad.position.set(0,1.0,0.115); pad.rotation.x=8*D; scene.add(pad);
  fixed(scene,0.035,V(0,0.92,0.17),V(0,0.40,0.26)); fixed(scene,0.04,V(0,0.40,0.26),V(0,0.03,0.30));
  // repose-pieds
  const fb=box(0.56,0.04,0.16,MAT.steel); fb.position.set(0,0.16,0.36); scene.add(fb); fixed(scene,0.03,V(0,0.14,0.36),V(0,0.03,0.36));
  // pivots des leviers + montants
  const piv={L:V(-0.46,0.34,0.28),R:V(0.46,0.34,0.28)};
  for(const s of [-1,1]){ fixed(scene,0.045,V(0.46*s,0.03,0.28),V(0.46*s,0.34,0.28)); fixed(scene,0.04,V(0.46*s,0.03,0.70),V(0.46*s,0.34,0.28)); const hub=cyl(0.06,0.06,0.10,MAT.dark); hub.rotation.z=Math.PI/2; hub.position.copy(piv[s<0?'L':'R']); scene.add(hub); }
  const parts={}; for(const k of ['L','R']) parts[k]={lev:rod(scene,0.032,MAT.steel), con:rod(scene,0.028,MAT.steel), grip:rod(scene,0.022,MAT.dark), horn:rod(scene,0.025,MAT.steel), plate:cyl(0.20,0.20,0.045,MAT.dark)};
  for(const k of ['L','R']){ parts[k].plate.rotation.z=Math.PI/2; scene.add(parts[k].plate); }
  const R=0.743, a0=Math.atan2(0.22,0.71), a1=Math.atan2(-0.28,0.69);   // angle du levier (0 = vertical), positif = vers l'avant
  return { man, pose(t){ const a=lerp(a0,a1,t); const h={};
    for(const [k,s] of [['L',-1],['R',1]]){ const P=piv[k], u=V(0,Math.cos(a),Math.sin(a)), H=P.clone().addScaledVector(u,R); H.x=0.22*s; h[k]=H;
      const Q=V(0.46*s,H.y+0.07,H.z); between(parts[k].lev,P,Q); between(parts[k].con,Q,H.clone().add(V(0,0.07,0))); between(parts[k].grip,H.clone().add(V(0,-0.08,0)),H.clone().add(V(0,0.08,0)));
      const hd=V(0,-Math.sin(a),Math.cos(a)); const he=P.clone().addScaledVector(hd,0.30).add(V(0.10*s,0,0)); between(parts[k].horn,P.clone().add(V(0.05*s,0,0)),he.clone().add(V(0.06*s,0,0))); parts[k].plate.position.copy(he); }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.19,0.21,0.34),R:V(0.19,0.21,0.34)}, elbowPole:{L:V(-0.25,-0.5,-1),R:V(0.25,-0.5,-1)}, kneePole:{L:V(-0.3,0.6,1),R:V(0.3,0.6,1)}, footDir:{L:V(-0.1,0,1),R:V(0.1,0,1)}}); }};
}};

EX["Reverse pec deck"]={ cam:{target:V(0,1.1,0.1),dist:3.4,h:1.6,ang:25}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.50,-0.05); const pad=box(0.32,0.55,0.07,MAT.pad); pad.position.set(0,1.05,0.20); scene.add(pad); fixed(scene,0.04,V(0,0.78,0.25),V(0,0,0.25));
  fixed(scene,0.04,V(0,0,0.32),V(0,2.0,0.32)); const arms=[rod(scene,0.03,MAT.steel),rod(scene,0.03,MAT.steel)], hd=[rod(scene,0.022,MAT.dark),rod(scene,0.022,MAT.dark)];
  const T=tiltVecs(5), pelvis=V(0,0.63,-0.10);
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const h={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().add(V(0.2*s,0,0)); const d=slerpDir(V(-s*0.15,-0.02,0.99),V(s*0.97,-0.02,-0.2),t); h[k]=S.clone().addScaledVector(d,0.56);
      between(arms[i],V(0.12*s,2.0,0.32),h[k].clone().add(V(0,0.10,0))); between(hd[i],h[k].clone().add(V(0,-0.07,0)),h[k].clone().add(V(0,0.10,0))); i++; }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:legsSeated(0.35), elbowPole:{L:V(-0.3,-0.4,-1),R:V(0.3,-0.4,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

EX["Shrugs haltères"]={ cam:{target:V(0,1.0,0),dist:3.2,h:1.3,ang:30}, build(scene){
  floor(scene); const man=Man(scene); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const sh=lerp(0,0.07,t); const h={L:V(-0.24,0.86+sh,0.02),R:V(0.24,0.86+sh,0.02)}; placeDB(dbs[0],h.L,V(0,0,1)); placeDB(dbs[1],h.R,V(0,0,1));
    man.set({...STAND, shrug:sh, hand:h, foot:feetStand(0.14), elbowPole:{L:V(-1,0,-0.3),R:V(1,0,-0.3)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};

// curls : coude fixe, avant-bras qui tourne dans le plan sagittal
function curlHands(E,fwdH,ang,len){ return E.clone().add(V(0,-len*Math.cos(ang),0)).addScaledVector(fwdH,len*Math.sin(ang)); }
EX["Curl haltères debout"]={ cam:{target:V(0,1.0,0),dist:3.2,h:1.3,ang:45}, build(scene){
  floor(scene); const man=Man(scene); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const a=lerp(5,135,t)*D; const el={L:V(-0.21,1.15,0.0),R:V(0.21,1.15,0.0)}, h={};
    for(const k of ['L','R']) h[k]=curlHands(el[k],V(0,0,1),a,0.30); placeDB(dbs[0],h.L,V(1,0,0)); placeDB(dbs[1],h.R,V(1,0,0));
    man.set({...STAND, hand:h, elbow:el, foot:feetStand(0.14), elbowPole:{L:V(0,0,-1),R:V(0,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};
EX["Curl marteau"]={ cam:{target:V(0,1.0,0),dist:3.2,h:1.3,ang:45}, build(scene){
  floor(scene); const man=Man(scene); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const a=lerp(5,130,t)*D; const el={L:V(-0.21,1.15,0.0),R:V(0.21,1.15,0.0)}, h={};
    for(const k of ['L','R']) h[k]=curlHands(el[k],V(0,0,1),a,0.30); const ax=V(0,Math.sin(a),Math.cos(a)); placeDB(dbs[0],h.L,ax); placeDB(dbs[1],h.R,ax);
    man.set({...STAND, hand:h, elbow:el, foot:feetStand(0.14), elbowPole:{L:V(0,0,-1),R:V(0,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};
EX["Curl inversé barre EZ"]={ cam:{target:V(0,1.0,0),dist:3.2,h:1.3,ang:45}, build(scene){
  floor(scene); const man=Man(scene); const bar=ezbar(scene);
  return { man, pose(t){ const a=lerp(5,120,t)*D; const el={L:V(-0.19,1.15,0.0),R:V(0.19,1.15,0.0)}, h={};
    for(const k of ['L','R']) h[k]=curlHands(el[k],V(0,0,1),a,0.30); bar(h.L.clone().add(h.R).multiplyScalar(0.5),V(1,0,0));
    man.set({...STAND, hand:h, elbow:el, foot:feetStand(0.14), elbowPole:{L:V(0,0,-1),R:V(0,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};
EX["Curl incliné haltères"]={ cam:{target:V(0,0.8,0),dist:3.1,h:1.35,ang:70}, build(scene){
  floor(scene); const man=Man(scene); const inc=52, hinge=V(0,0.44,0.25); bench(scene,inc,hinge); const F=benchFrame(inc,hinge); const dbs=[dumbbell(scene),dumbbell(scene)];
  const C=F.pelvis.clone().addScaledVector(F.up,0.5), right=frameOf(F.up,F.fwd);
  return { man, pose(t){ const a=lerp(5,125,t)*D; const h={}, el={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().addScaledVector(right,0.2*s); el[k]=S.clone().add(V(0.03*s,-0.29,0)); h[k]=curlHands(el[k],V(0,0,1),a,0.30); placeDB(dbs[i++],h[k],V(1,0,0)); }
    man.set({pelvis:F.pelvis, up:F.up, fwd:F.fwd, hand:h, elbow:el, foot:legsSeated(F.pelvis.z+0.52), elbowPole:{L:V(0,0,-1),R:V(0,0,-1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};
EX["Curl pupitre"]={ cam:{target:V(0,0.95,0.15),dist:3.2,h:1.35,ang:65}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.52,-0.05);
  const pad=box(0.5,0.05,0.36,MAT.pad); const pc=V(0,0.98,0.22); pad.position.copy(pc); pad.rotation.x=45*D; scene.add(pad); fixed(scene,0.04,V(0,0.88,0.30),V(0,0,0.30));
  const bar=ezbar(scene); const pelvis=V(0,0.65,-0.10), T=tiltVecs(14);
  return { man, pose(t){ const C=pelvis.clone().addScaledVector(T.up,0.5); const h={}, el={}; const a=lerp(0,115,t)*D;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().add(V(0.2*s,0,0)); el[k]=V(0.17*s,0.92,0.33); const ua=el[k].clone().sub(S).normalize();
      const n=V(0,0.707,0.707).normalize(); h[k]=el[k].clone().addScaledVector(ua,0.30*Math.cos(a)).addScaledVector(n,0.30*Math.sin(a)); }
    bar(h.L.clone().add(h.R).multiplyScalar(0.5),V(1,0,0));
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, elbow:el, foot:legsSeated(0.40), elbowPole:{L:V(0,0,1),R:V(0,0,1)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};
EX["Spider curl"]={ cam:{target:V(0,0.8,0.25),dist:3.2,h:1.3,ang:75}, build(scene){
  const P=proneSetup(scene,42); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man:P.man, pose(t){ const right=frameOf(P.up,P.fwd), C=P.pelvis.clone().addScaledVector(P.up,0.5); const h={}, el={}; let i=0; const a=lerp(5,130,t)*D;
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().addScaledVector(right,0.2*s); el[k]=S.clone().add(V(0,-0.29,0.03)); h[k]=curlHands(el[k],V(0,0,1),a,0.30); placeDB(dbs[i++],h[k],V(1,0,0)); }
    P.man.set({pelvis:P.pelvis, up:P.up, fwd:P.fwd, hand:h, elbow:el, foot:P.foot, elbowPole:{L:V(0,0,-1),R:V(0,0,-1)}, kneePole:{L:V(0,0,1),R:V(0,0,1)}}); }};
}};

// ---------- LEGS ----------
EX["Squat Smith"]={ cam:{target:V(0,0.95,0),dist:3.6,h:1.35,ang:60}, build(scene){
  floor(scene); const man=Man(scene); const zb=-0.05; const setBar=smithFrame(scene,zb);
  return { man, pose(t){ const tilt=lerp(4,26,t), T=tiltVecs(tilt), a=tilt*D; const py=lerp(0.93,0.50,t);
    const pelvis=V(0,py,zb-0.53*Math.sin(a)+0.09*Math.cos(a)); const barC=pelvis.clone().addScaledVector(T.up,0.53).addScaledVector(T.fwd,-0.09); setBar(barC.y);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.43,barC.y,zb),R:V(0.43,barC.y,zb)}, foot:{L:V(-0.18,0.07,zb+0.16),R:V(0.18,0.07,zb+0.16)},
      elbowPole:{L:V(-0.3,-1,-0.6),R:V(0.3,-1,-0.6)}, kneePole:{L:V(-0.35,0,1),R:V(0.35,0,1)}, footDir:{L:V(-0.25,0,1),R:V(0.25,0,1)}}); }};
}};

EX["Hack squat"]={ cam:{target:V(0,0.85,0),dist:3.6,h:1.3,ang:72}, build(scene){
  floor(scene); const man=Man(scene); const d=V(0,Math.cos(36*D),-Math.sin(36*D)), nrm=V(0,Math.sin(36*D),Math.cos(36*D));
  const Q=V(0,0.98,-0.12).addScaledVector(nrm,-0.27); for(const x of [-0.17,0.17]) fixed(scene,0.04,Q.clone().add(V(x,0,0)).addScaledVector(d,-0.75),Q.clone().add(V(x,0,0)).addScaledVector(d,1.0));
  fixed(scene,0.04,Q.clone().addScaledVector(d,-0.75).add(V(-0.17,0,0)),V(-0.17,0,Q.clone().addScaledVector(d,-0.75).z)); fixed(scene,0.04,Q.clone().addScaledVector(d,-0.75).add(V(0.17,0,0)),V(0.17,0,Q.clone().addScaledVector(d,-0.75).z));
  const plat=box(0.6,0.05,0.45,MAT.steel); plat.position.set(0,0.17,0.40); plat.rotation.x=-14*D; scene.add(plat); const base=box(0.7,0.05,1.6,MAT.steel); base.position.set(0,0.025,0); scene.add(base);
  const back=box(0.36,0.8,0.07,MAT.pad), sh=[cyl(0.05,0.05,0.18,MAT.pad),cyl(0.05,0.05,0.18,MAT.pad)]; scene.add(back); sh.forEach(x=>{x.rotation.x=Math.PI/2; scene.add(x);});
  const P0=V(0,0.98,-0.12); const right=V(1,0,0);
  return { man, pose(t){ const pelvis=P0.clone().addScaledVector(d,-lerp(0.0,0.40,t)); const up=d.clone(), fwd=nrm.clone();
    back.position.copy(pelvis.clone().addScaledVector(up,0.35).addScaledVector(fwd,-0.17)); back.quaternion.copy(basisQuat(right,up,fwd));
    const C=pelvis.clone().addScaledVector(up,0.5); sh[0].position.copy(C.clone().add(V(-0.13,0.08,0)).addScaledVector(fwd,0.02)); sh[1].position.copy(C.clone().add(V(0.13,0.08,0)).addScaledVector(fwd,0.02));
    const hd={L:C.clone().add(V(-0.25,0.10,0)).addScaledVector(fwd,0.10),R:C.clone().add(V(0.25,0.10,0)).addScaledVector(fwd,0.10)};
    man.set({pelvis, up, fwd, hand:hd, foot:{L:V(-0.17,0.26,0.36),R:V(0.17,0.26,0.36)}, elbowPole:{L:V(-1,-0.5,0),R:V(1,-0.5,0)}, kneePole:{L:V(-0.3,0.3,1),R:V(0.3,0.3,1)}, footDir:{L:V(-0.2,0.24,1),R:V(0.2,0.24,1)}}); }};
}};

EX["Presse à cuisses"]={ cam:{target:V(0,0.8,0.35),dist:3.8,h:1.4,ang:78}, build(scene){
  floor(scene); const man=Man(scene); const s=V(0,Math.sin(45*D),Math.cos(45*D)), pn=V(0,Math.cos(45*D),-Math.sin(45*D));
  const T={up:V(0,Math.sin(48*D),-Math.cos(48*D)),fwd:V(0,Math.cos(48*D),Math.sin(48*D))}; const pelvis=V(0,0.60,-0.05); const right=V(1,0,0);
  // dossier collé au dos (épaisseur du buste ~0.14) et assise sous les fessiers
  const back=box(0.44,0.78,0.08,MAT.pad); back.position.copy(pelvis.clone().addScaledVector(T.up,0.36).addScaledVector(T.fwd,-0.185)); back.quaternion.copy(basisQuat(right,T.up,T.fwd)); scene.add(back);
  const sUp=V(0,Math.cos(14*D),Math.sin(14*D)), sF=V(0,-Math.sin(14*D),Math.cos(14*D));
  const seat=box(0.44,0.08,0.40,MAT.pad); seat.position.copy(pelvis.clone().add(V(0,-0.155,0.13))); seat.quaternion.copy(basisQuat(right,sUp,sF)); scene.add(seat);
  const bk=back.position.clone().addScaledVector(T.fwd,-0.05); fixed(scene,0.04,bk,V(0,0.05,bk.z)); fixed(scene,0.04,seat.position.clone().add(V(0,-0.05,0)),V(0,0.05,seat.position.z));
  const base=box(0.8,0.05,2.2,MAT.steel); base.position.set(0,0.025,0.5); scene.add(base);
  for(const x of [-0.42,0.42]) fixed(scene,0.045,V(x,0.25,0.15),V(x,0.25,0.15).addScaledVector(s,2.0));
  const sled=box(0.7,0.06,0.55,MAT.steel); scene.add(sled); const pls=[cyl(0.22,0.22,0.05,MAT.dark),cyl(0.22,0.22,0.05,MAT.dark)]; pls.forEach(p=>scene.add(p));
  return { man, pose(t){ const hip=pelvis.clone().add(V(0,-0.04,0.02)); const L=lerp(0.80,0.42,t); const ft=hip.clone().addScaledVector(s,L).add(V(0,0.02,0.02));
    sled.position.copy(ft.clone().addScaledVector(s,0.06)); sled.quaternion.copy(basisQuat(V(1,0,0),pn,s));
    pls.forEach((p,i)=>{ p.position.copy(ft.clone().addScaledVector(s,0.10).add(V(i?0.47:-0.47,0,0))); p.rotation.set(0,0,Math.PI/2); });
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:pelvis.clone().add(V(-0.29,-0.02,0.12)),R:pelvis.clone().add(V(0.29,-0.02,0.12))}, foot:{L:ft.clone().add(V(-0.15,0,0)),R:ft.clone().add(V(0.15,0,0))},
      elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(-0.2,0.7,-0.7),R:V(0.2,0.7,-0.7)}, footDir:{L:pn.clone().add(V(-0.15,0,0)),R:pn.clone().add(V(0.15,0,0))}}); }};
}};

function legMachine(scene){ floor(scene); const man=Man(scene); seatBox(scene,0.50,0.12,0.42); const back=box(0.4,0.65,0.07,MAT.pad); back.position.set(0,0.92,-0.15); back.rotation.x=-10*D; scene.add(back);
  for(const x of [-0.3,0.3]) fixed(scene,0.04,V(x,0,0.38),V(x,0.62,0.38)); return man; }
EX["Leg extension"]={ cam:{target:V(0,0.6,0.2),dist:3.2,h:1.15,ang:78}, build(scene){
  const man=legMachine(scene); const T=tiltVecs(-10), pelvis=V(0,0.63,-0.03); const pad=cyl(0.05,0.05,0.36,MAT.pad); pad.rotation.z=Math.PI/2; scene.add(pad); const lev=rod(scene,0.03,MAT.steel), arm=rod(scene,0.025,MAT.steel);
  return { man, pose(t){ const K=V(0,0.60,0.40), th=lerp(0,82,t)*D; const d=V(0,-Math.cos(th),Math.sin(th)), n=V(0,Math.sin(th),Math.cos(th)); const ank=K.clone().addScaledVector(d,0.42);
    // le boudin repose sur le devant du tibia, au-dessus de la cheville (pas sur le pied)
    pad.position.copy(K.clone().addScaledVector(d,0.33).addScaledVector(n,0.10)); const pv=V(0.30,0.60,0.40); const pe=pad.position.clone().add(V(0.19,0,0));
    between(lev,pv,pv.clone().addScaledVector(d,0.33).addScaledVector(n,0.10)); between(arm,pv.clone().addScaledVector(d,0.33).addScaledVector(n,0.10),pe);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.28,0.58,0.05),R:V(0.28,0.58,0.05)}, foot:{L:ank.clone().add(V(-0.11,0,0)),R:ank.clone().add(V(0.11,0,0))},
      elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(0,1,0.2),R:V(0,1,0.2)}, footDir:{L:n.clone().addScaledVector(d,-0.15),R:n.clone().addScaledVector(d,-0.15)}}); }};
}};
EX["Leg curl assis"]={ cam:{target:V(0,0.6,0.2),dist:3.2,h:1.15,ang:78}, build(scene){
  const man=legMachine(scene); const T=tiltVecs(-14), pelvis=V(0,0.63,-0.03); const tp=cyl(0.055,0.055,0.40,MAT.pad); tp.rotation.z=Math.PI/2; tp.position.set(0,0.76,0.30); scene.add(tp);
  const pad=cyl(0.05,0.05,0.36,MAT.pad); pad.rotation.z=Math.PI/2; scene.add(pad); const lev=rod(scene,0.03,MAT.steel);
  return { man, pose(t){ const K=V(0,0.60,0.40), th=lerp(12,100,t)*D; const ank=K.clone().add(V(0,-0.42*Math.sin(th),0.42*Math.cos(th)));
    pad.position.copy(ank.clone().add(V(0,-0.06*Math.cos(th),0.06*Math.sin(th)*-1).add(V(0,-0.02,-0.0)))); between(lev,V(0.30,0.60,0.40),pad.position.clone().add(V(0.18,0,0)));
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.28,0.58,0.05),R:V(0.28,0.58,0.05)}, foot:{L:ank.clone().add(V(-0.11,0,0)),R:ank.clone().add(V(0.11,0,0))},
      elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(0,1,0.2),R:V(0,1,0.2)}, footDir:{L:V(0,0.3,1),R:V(0,0.3,1)}}); }};
}};
EX["Leg curl allongé"]={ cam:{target:V(0,0.7,0.2),dist:3.4,h:1.3,ang:80}, build(scene){
  floor(scene); const man=Man(scene); const bench2=box(0.36,0.08,1.25,MAT.pad); bench2.position.set(0,0.64,-0.18); scene.add(bench2); fixed(scene,0.04,V(0,0.6,-0.5),V(0,0,-0.5)); fixed(scene,0.04,V(0,0.6,0.3),V(0,0,0.3));
  for(const x of [-0.25,0.25]) fixed(scene,0.02,V(x,0.62,-0.72),V(x,0.68,-0.72)); const pad=cyl(0.05,0.05,0.36,MAT.pad); pad.rotation.z=Math.PI/2; scene.add(pad); const lev=rod(scene,0.03,MAT.steel);
  const pelvis=V(0,0.80,0.18), up=V(0,0,-1), fwd=V(0,-1,0);
  return { man, pose(t){ const K=V(0,0.74,0.58), th=lerp(4,105,t)*D; const ank=K.clone().add(V(0,0.42*Math.sin(th),0.42*Math.cos(th)));
    pad.position.copy(ank.clone().add(V(0,0.06*Math.cos(th),-0.06*Math.sin(th)))); between(lev,V(0.30,0.74,0.58),pad.position.clone().add(V(0.18,0,0)));
    man.set({pelvis, up, fwd, hand:{L:V(-0.25,0.70,-0.72),R:V(0.25,0.70,-0.72)}, foot:{L:ank.clone().add(V(-0.10,0,0)),R:ank.clone().add(V(0.10,0,0))},
      elbowPole:{L:V(-0.4,-1,0),R:V(0.4,-1,0)}, kneePole:{L:V(0,-1,0),R:V(0,-1,0)}, footDir:{L:V(0,-Math.cos(th),Math.sin(th)),R:V(0,-Math.cos(th),Math.sin(th))}}); }};
}};

function hinge(scene,smith){ floor(scene); const man=Man(scene); const zf=0.0, zb=0.13; let setBar=null, dbs=null;
  if(smith) setBar=smithFrame(scene,zb); else dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const tilt=lerp(3,74,t), T=tiltVecs(tilt); const pelvis=V(0,lerp(0.915,0.86,t),lerp(zf-0.02,zf-0.30,t));
    const C=pelvis.clone().addScaledVector(T.up,0.5); const L=0.575; const hz=zb; const h={};
    for(const [k,s] of [['L',-1],['R',1]]){ const S=C.clone().add(V(0.2*s,0,0)); const dz=hz-S.z, dx=0.03*s; const hy=S.y-Math.sqrt(Math.max(0.01,L*L-dz*dz-dx*dx)); h[k]=V(S.x-dx,hy,hz); }
    if(smith){ setBar(h.L.y); h.L.x=-0.22; h.R.x=0.22; } else { h.L.x=-0.17; h.R.x=0.17; placeDB(dbs[0],h.L,V(1,0,0)); placeDB(dbs[1],h.R,V(1,0,0)); }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.13,0.07,zf-0.03),R:V(0.13,0.07,zf-0.03)}, elbowPole:{L:V(-0.3,0,-1),R:V(0.3,0,-1)}, kneePole:{L:V(-0.1,0,1),R:V(0.1,0,1)}, footDir:{L:V(-0.08,0,1),R:V(0.08,0,1)}}); }}; }
EX["Soulevé de terre Smith"]={ cam:{target:V(0,0.85,0),dist:3.7,h:1.3,ang:72}, build(scene){ return hinge(scene,true); }};
EX["Soulevé de terre roumain haltères"]={ cam:{target:V(0,0.85,0),dist:3.4,h:1.3,ang:72}, build(scene){ return hinge(scene,false); }};

EX["Fentes bulgares"]={ cam:{target:V(0,0.8,0),dist:3.4,h:1.25,ang:75}, build(scene){
  floor(scene); const man=Man(scene); const b=box(0.30,0.07,0.6,MAT.pad); b.position.set(0,0.44,-0.78); scene.add(b); fixed(scene,0.035,V(0,0.4,-0.78),V(0,0,-0.78)); const dbs=[dumbbell(scene),dumbbell(scene)];
  return { man, pose(t){ const T=tiltVecs(lerp(8,16,t)), pelvis=V(0,lerp(0.90,0.56,t),lerp(-0.12,-0.16,t)); const C=pelvis.clone().addScaledVector(T.up,0.5);
    const h={L:C.clone().add(V(-0.25,-0.58,0.03)),R:C.clone().add(V(0.25,-0.58,0.03))}; placeDB(dbs[0],h.L,V(0,0,1)); placeDB(dbs[1],h.R,V(0,0,1));
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.12,0.07,0.38),R:V(0.12,0.52,-0.62)}, elbowPole:{L:V(-1,0,-0.3),R:V(1,0,-0.3)}, kneePole:{L:V(-0.1,0,1),R:V(0,-1,0.3)}, footDir:{L:V(-0.1,0,1),R:V(0,-0.2,-1)}}); }};
}};
EX["Fentes marchées haltères"]={ loop:6.0, cam:{target:V(0,0.75,0),dist:4.3,h:1.3,ang:78}, build(scene){
  floor(scene); const man=Man(scene); const dbs=[dumbbell(scene),dumbbell(scene)]; const SL=0.85;
  function step(s,z0){ // s : 0..1 d'un pas ; renvoie bassin, pied avant, pied arrière (repères relatifs)
    let pz,py,lead,trail,tdir;
    if(s<0.35){ const a=ease(s/0.35); pz=lerp(0,0.42,a); py=lerp(0.93,0.82,a); lead=V(0,0.07+0.13*Math.sin(Math.PI*a),lerp(0,SL,a)); trail=V(0,lerp(0.07,0.09,a),0); tdir=slerpDir(V(0,0,1),V(0,-0.35,1),a); }
    else if(s<0.60){ const b=ease((s-0.35)/0.25); pz=lerp(0.42,0.45,b); py=lerp(0.82,0.56,b); lead=V(0,0.07,SL); trail=V(0,lerp(0.09,0.11,b),0); tdir=slerpDir(V(0,-0.35,1),V(0,-0.62,1),b); }
    else if(s<0.80){ const c=ease((s-0.60)/0.20); pz=lerp(0.45,0.62,c); py=lerp(0.56,0.80,c); lead=V(0,0.07,SL); trail=V(0,lerp(0.11,0.16,c),0); tdir=slerpDir(V(0,-0.62,1),V(0,-1,0.55),c); }
    else { const d=ease((s-0.80)/0.20); pz=lerp(0.62,SL,d); py=lerp(0.80,0.93,d); lead=V(0,0.07,SL); trail=V(0,lerp(0.16,0.07,d)+0.12*Math.sin(Math.PI*d),lerp(0,SL,d)); tdir=slerpDir(V(0,-1,0.55),V(0,0,1),d); }
    return {pelvis:V(0,py,z0+pz), lead:lead.add(V(0,0,z0)), trail:trail.add(V(0,0,z0)), tdir, low:s>=0.35&&s<0.80};
  }
  return { man, pose(t){ const first=t<0.5, s=first?t*2:(t-0.5)*2, z0=first?-SL:0; const r=step(s,z0);
    const LK=first?'L':'R', TK=first?'R':'L'; const foot={}, fdir={}, kp={};
    foot[LK]=r.lead.clone().setX(LK==='L'?-0.12:0.12); foot[TK]=r.trail.clone().setX(TK==='L'?-0.12:0.12);
    fdir[LK]=V(0,0,1); fdir[TK]=r.tdir; kp[LK]=V(0,0.3,1); kp[TK]=V(0,-1,0.35);
    const T=tiltVecs(4), C=r.pelvis.clone().addScaledVector(T.up,0.5);
    const h={L:C.clone().add(V(-0.25,-0.58,0.02)),R:C.clone().add(V(0.25,-0.58,0.02))}; placeDB(dbs[0],h.L,V(0,0,1)); placeDB(dbs[1],h.R,V(0,0,1));
    man.set({pelvis:r.pelvis, up:T.up, fwd:T.fwd, hand:h, foot, elbowPole:{L:V(-1,0,-0.3),R:V(1,0,-0.3)}, kneePole:kp, footDir:fdir}); }};
}};

// Soulevé de terre roumain : genoux légèrement fléchis et fixes, hanches vers l'arrière, dos plat, charge qui frôle les jambes

function adMachine(scene,abduct){ floor(scene); const man=Man(scene); seatBox(scene,0.50,0.05,0.42); const back=box(0.4,0.65,0.07,MAT.pad); back.position.set(0,0.95,-0.18); back.rotation.x=-12*D; scene.add(back);
  const pads=[box(0.06,0.22,0.16,MAT.pad),box(0.06,0.22,0.16,MAT.pad)]; pads.forEach(p=>scene.add(p)); const fr=[rod(scene,0.03,MAT.steel),rod(scene,0.03,MAT.steel)];
  const T=tiltVecs(-12), pelvis=V(0,0.63,-0.02);
  return { man, pose(t){ const ph=(abduct?lerp(6,38,t):lerp(38,6,t))*D; const f={}; let i=0;
    for(const [k,s] of [['L',-1],['R',1]]){ const hip=V(0.1*s,0.59,-0.0); const K=hip.clone().add(V(s*0.43*Math.sin(ph),0.0,0.43*Math.cos(ph))); f[k]=K.clone().add(V(s*0.05,-0.36,0.06));
      pads[i].position.copy(K.clone().add(V(s*(abduct?0.08:-0.08),0,-0.04))); pads[i].rotation.y=s*ph; between(fr[i],pads[i].position.clone().add(V(0,-0.1,0)),V(0.12*s,0.30,0.20)); i++; }
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.27,0.58,0.12),R:V(0.27,0.58,0.12)}, foot:f, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(-0.3,1,0.3),R:V(0.3,1,0.3)}, footDir:toesOut}); }}; }
EX["Adducteurs machine"]={ cam:{target:V(0,0.7,0.2),dist:3.2,h:1.6,ang:20}, build(scene){ return adMachine(scene,false); }};
EX["Abducteurs machine"]={ cam:{target:V(0,0.7,0.2),dist:3.2,h:1.6,ang:20}, build(scene){ return adMachine(scene,true); }};

EX["Crunch poulie"]={ cam:{target:V(0,0.7,0.2),dist:3.4,h:1.25,ang:75}, build(scene){
  floor(scene); const man=Man(scene); const P=tower(scene,0,1.12,2.05,0); const c=cable(scene), r1=rod(scene,0.012,MAT.rope), r2=rod(scene,0.012,MAT.rope);
  const pelvis=V(0,0.55,-0.02);
  return { man, pose(t){ const T=tiltVecs(lerp(15,62,t)); const right=frameOf(T.up,T.fwd); const head=pelvis.clone().addScaledVector(T.up,0.73);
    const h={L:head.clone().addScaledVector(right,-0.13).addScaledVector(T.fwd,0.04).addScaledVector(T.up,-0.04),R:head.clone().addScaledVector(right,0.13).addScaledVector(T.fwd,0.04).addScaledVector(T.up,-0.04)};
    const knot=head.clone().addScaledVector(T.up,0.15).addScaledVector(T.fwd,0.05); between(c,P,knot); between(r1,knot,h.L); between(r2,knot,h.R);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.12,0.07,-0.44),R:V(0.12,0.07,-0.44)}, elbowPole:{L:V(-0.3,-1,1),R:V(0.3,-1,1)}, kneePole:{L:V(0,-1,1),R:V(0,-1,1)}, footDir:{L:V(0,-0.4,-1),R:V(0,-0.4,-1)}}); }};
}};
EX["Crunch machine"]={ cam:{target:V(0,0.95,0.1),dist:3.3,h:1.35,ang:72}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.50,0.0); const roll=cyl(0.05,0.05,0.4,MAT.pad); roll.rotation.z=Math.PI/2; roll.position.set(0,0.22,0.38); scene.add(roll);
  const back=box(0.36,0.6,0.07,MAT.pad); scene.add(back); const arms=[rod(scene,0.03,MAT.steel),rod(scene,0.03,MAT.steel)]; const pelvis=V(0,0.63,-0.03);
  return { man, pose(t){ const T=tiltVecs(lerp(-4,36,t)); const right=frameOf(T.up,T.fwd); back.position.copy(pelvis.clone().addScaledVector(T.up,0.38).addScaledVector(T.fwd,-0.17)); back.quaternion.copy(basisQuat(right,T.up,T.fwd));
    const C=pelvis.clone().addScaledVector(T.up,0.5); const h={L:C.clone().addScaledVector(right,-0.16).addScaledVector(T.fwd,0.10).addScaledVector(T.up,0.10),R:C.clone().addScaledVector(right,0.16).addScaledVector(T.fwd,0.10).addScaledVector(T.up,0.10)};
    between(arms[0],h.L,h.L.clone().addScaledVector(T.up,0.22).addScaledVector(T.fwd,-0.2)); between(arms[1],h.R,h.R.clone().addScaledVector(T.up,0.22).addScaledVector(T.fwd,-0.2));
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:h, foot:{L:V(-0.12,0.16,0.34),R:V(0.12,0.16,0.34)}, elbowPole:{L:V(-0.5,-1,0.6),R:V(0.5,-1,0.6)}, kneePole:kneeFwd}); }};
}};
EX["Relevés de jambes suspendu"]={ cam:{target:V(0,1.4,0),dist:4.2,h:1.6,ang:70}, build(scene){
  floor(scene); const man=Man(scene); const by=2.25; fixed(scene,0.018,V(-0.7,by,0),V(0.7,by,0)); for(const x of [-0.7,0.7]) fixed(scene,0.04,V(x,0,0),V(x,by+0.1,0));
  return { man, pose(t){ const T=tiltVecs(lerp(0,-12,t)); const C=V(0,by-0.555,lerp(0,-0.04,t)), pelvis=C.clone().addScaledVector(T.up,-0.5); const ph=lerp(0,88,t)*D;
    const f={L:pelvis.clone().add(V(-0.1,-0.86*Math.cos(ph),0.86*Math.sin(ph))),R:pelvis.clone().add(V(0.1,-0.86*Math.cos(ph),0.86*Math.sin(ph)))};
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.32,by,0),R:V(0.32,by,0)}, foot:f, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(0,Math.sin(ph),Math.cos(ph)),R:V(0,Math.sin(ph),Math.cos(ph))}, footDir:{L:V(0,Math.sin(ph)*0.9+0.1,Math.cos(ph)),R:V(0,Math.sin(ph)*0.9+0.1,Math.cos(ph))}}); }};
}};
EX["Relevés de genoux chaise romaine"]={ cam:{target:V(0,1.2,0),dist:3.8,h:1.5,ang:72}, build(scene){
  floor(scene); const man=Man(scene); const back=box(0.36,0.75,0.07,MAT.pad); back.position.set(0,1.32,-0.17); scene.add(back);
  for(const x of [-0.24,0.24]){ const ap=box(0.1,0.06,0.42,MAT.pad); ap.position.set(x,1.30,0.06); scene.add(ap); fixed(scene,0.035,V(x,0,-0.2),V(x,1.27,-0.2)); fixed(scene,0.03,V(x,1.27,0.25),V(x,1.36,0.25)); }
  fixed(scene,0.04,V(0,0,-0.24),V(0,1.75,-0.24));
  return { man, pose(t){ const pelvis=V(0,lerp(1.15,1.18,t),-0.06); const up=V(0,1,0), fwd=V(0,0,1); const ph=lerp(0,105,t)*D;
    const K={L:pelvis.clone().add(V(-0.1,-0.04-0.43*Math.cos(ph),0.43*Math.sin(ph))),R:pelvis.clone().add(V(0.1,-0.04-0.43*Math.cos(ph),0.43*Math.sin(ph)))};
    const f={L:K.L.clone().add(V(0,-0.41,0.06)),R:K.R.clone().add(V(0,-0.41,0.06))};
    man.set({pelvis, up, fwd, hand:{L:V(-0.24,1.37,0.25),R:V(0.24,1.37,0.25)}, elbow:{L:V(-0.24,1.36,-0.02),R:V(0.24,1.36,-0.02)}, foot:f, elbowPole:{L:V(0,-1,0),R:V(0,-1,0)}, kneePole:{L:V(0,0.3,1),R:V(0,0.3,1)}, footDir:{L:V(0,-0.3,1),R:V(0,-0.3,1)}}); }};
}};
EX["Mollets debout"]={ cam:{target:V(0,0.9,0),dist:3.4,h:1.3,ang:75}, build(scene){
  floor(scene); const man=Man(scene); const step=box(0.5,0.12,0.18,MAT.steel); step.position.set(0,0.06,0.10); scene.add(step);
  for(const x of [-0.45,0.45]) fixed(scene,0.04,V(x,0,-0.2),V(x,1.9,-0.2)); const pads=[cyl(0.05,0.05,0.16,MAT.pad),cyl(0.05,0.05,0.16,MAT.pad)]; pads.forEach(p=>{p.rotation.x=Math.PI/2; scene.add(p);}); const yoke=rod(scene,0.03,MAT.steel);
  return { man, pose(t){ const b=lerp(-22,32,t)*D; const ball={L:V(-0.12,0.15,0.13),R:V(0.12,0.15,0.13)}; const A={}, fd={};
    for(const k of ['L','R']){ A[k]=ball[k].clone().add(V(0,0.15*Math.sin(b),-0.15*Math.cos(b))); fd[k]=ball[k].clone().sub(A[k]).normalize(); }
    const pelvis=V(0,A.L.y+0.85,A.L.z+0.02); const C=pelvis.clone().add(V(0,0.5,0)); pads[0].position.copy(C.clone().add(V(-0.13,0.09,0))); pads[1].position.copy(C.clone().add(V(0.13,0.09,0)));
    between(yoke,C.clone().add(V(-0.45,0.11,-0.02)),C.clone().add(V(0.45,0.11,-0.02)));
    man.set({pelvis, up:V(0,1,0), fwd:V(0,0,1), hand:{L:C.clone().add(V(-0.30,0.12,0.05)),R:C.clone().add(V(0.30,0.12,0.05))}, foot:A, elbowPole:{L:V(-1,-1,0),R:V(1,-1,0)}, kneePole:kneeFwd, footDir:fd}); }};
}};
EX["Mollets assis"]={ cam:{target:V(0,0.5,0.15),dist:3.0,h:1.1,ang:78}, build(scene){
  floor(scene); const man=Man(scene); seatBox(scene,0.48,-0.02); const plat=box(0.5,0.1,0.16,MAT.steel); plat.position.set(0,0.11,0.50); scene.add(plat);
  const kp=box(0.42,0.06,0.16,MAT.pad); scene.add(kp); fixed(scene,0.035,V(0.3,0.1,0.50),V(0.3,0.75,0.38)); const pelvis=V(0,0.61,-0.04);
  return { man, pose(t){ const b=lerp(-22,30,t)*D; const ball={L:V(-0.12,0.18,0.53),R:V(0.12,0.18,0.53)}; const A={}, fd={};
    for(const k of ['L','R']){ A[k]=ball[k].clone().add(V(0,0.15*Math.sin(b),-0.15*Math.cos(b))); fd[k]=ball[k].clone().sub(A[k]).normalize(); }
    const J=man.set({pelvis, up:V(0,1,0), fwd:V(0,0,1), hand:{L:V(-0.18,0.71,0.28),R:V(0.18,0.71,0.28)}, foot:A, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(0,1,0.2),R:V(0,1,0.2)}, footDir:fd});
    kp.position.copy(J.KL.clone().add(J.KR).multiplyScalar(0.5).add(V(0,0.09,-0.06))); }};
}};

// =====================================================================
//  ÉTIREMENTS ET MOBILITÉ (côté droit montré pour les étirements unilatéraux)
// =====================================================================
function wall(scene,x,z,w,dir){ const b=box(dir==='x'?0.06:(w||1.4),2.2,dir==='x'?(w||1.4):0.06,MAT.floor); b.position.set(x,1.1,z); scene.add(b); }
const breath=t=>t; // t va de 0 à 1 : on accentue légèrement l'étirement
EX["Étirement pectoral à la porte"]={ cam:{target:V(0.1,1.1,0),dist:3.3,h:1.6,ang:-35}, build(scene){
  floor(scene); const man=Man(scene); fixed(scene,0.05,V(0.52,0,0.02),V(0.52,2.1,0.02)); fixed(scene,0.05,V(-0.52,0,0.02),V(-0.52,2.1,0.02)); fixed(scene,0.05,V(-0.52,2.1,0.02),V(0.52,2.1,0.02));
  return { man, pose(t){ const pz=lerp(0.08,0.18,t); const pelvis=V(-0.02,0.93,pz), T={up:V(0,1,0),fwd:V(-0.25,0,1)};
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.24,0.92,pz+0.05),R:V(0.46,1.72,0.02)}, elbow:{R:V(0.46,1.43,0.02)}, foot:{L:V(-0.15,0.07,pz+0.25),R:V(0.12,0.07,pz-0.20)}, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:kneeFwd, footDir:toesOut}); }}; }};
EX["Étirement épaule bras croisé"]={ cam:{target:V(0,1.15,0),dist:3.0,h:1.4,ang:20}, build(scene){
  floor(scene); const man=Man(scene);
  return { man, pose(t){ const hr=V(lerp(-0.22,-0.30,t),1.37,lerp(0.24,0.20,t));
    man.set({...STAND, hand:{L:V(-0.02,1.33,0.26),R:hr}, foot:feetStand(0.14), elbowPole:{L:V(-0.3,-1,0.4),R:V(0,-1,0.6)}, kneePole:kneeFwd, footDir:toesOut}); }}; }};
EX["Étirement triceps au-dessus de la tête"]={ cam:{target:V(0,1.3,0),dist:3.0,h:1.5,ang:40}, build(scene){
  floor(scene); const man=Man(scene);
  return { man, pose(t){ const E=V(lerp(0.12,0.08,t),1.72,-0.02); const H=V(lerp(0.0,-0.04,t),1.50,-0.12);
    man.set({...STAND, hand:{L:E.clone().add(V(-0.03,0.04,0.03)),R:H}, elbow:{R:E}, foot:feetStand(0.14), elbowPole:{L:V(-1,0.5,0.5),R:V(0,1,0)}, kneePole:kneeFwd, footDir:toesOut}); }}; }};
EX["Livre ouvert (rotation thoracique)"]={ cam:{target:V(-0.15,0.2,0.0),dist:2.5,h:2.2,ang:260}, build(scene){
  floor(scene); const man=Man(scene); const fwd=V(-1,0,0);
  return { man, pose(t){ const e=ease(t), tw=lerp(0,140,e)*D, up=V(0,lerp(0.20,0.06,e),-1).normalize(), pelvis=V(0,0.17,0.25);
    const base={pelvis, up, fwd, twist:TW*tw, elbowPole:{L:V(0,-1,0),R:V(0,1,0)}, kneePole:{L:V(-1,-0.2,0),R:V(-1,0.2,0)}, footDir:{L:V(0,0,1),R:V(0,0,1)},
      foot:{L:V(-0.42,0.07,0.66),R:V(-0.42,0.20,0.66)}};
    const J=man.set({...base, hand:{L:V(-0.6,0.07,-0.2),R:V(-0.6,0.2,-0.2)}});
    const hl=V(-0.57,0.07,-0.27), d0=hl.clone().add(V(0,0.09,0)).sub(J.SR).normalize();
    const hr=J.SR.clone().addScaledVector(slerpDir(d0,J.cn,Math.min(1,e*1.4)),0.57); if(hr.y<0.07) hr.y=0.07;
    man.set({...base, hand:{L:hl,R:hr}}); }};
}};
EX["Suspension passive"]={ cam:{target:V(0,1.4,0),dist:4.0,h:1.6,ang:35}, build(scene){
  floor(scene); const man=Man(scene); const by=2.25; fixed(scene,0.018,V(-0.7,by,0),V(0.7,by,0)); for(const x of [-0.7,0.7]) fixed(scene,0.04,V(x,0,0),V(x,by+0.1,0));
  return { man, pose(t){ const sh=lerp(0.03,0.06,t); const C=V(0,by-0.555-sh+0.02,0), pelvis=C.clone().add(V(0,-0.5,0));
    man.set({pelvis, up:V(0,1,0), fwd:V(0,0,1), shrug:sh, hand:{L:V(-0.32,by,0),R:V(0.32,by,0)}, foot:{L:pelvis.clone().add(V(-0.11,-0.85,0.05)),R:pelvis.clone().add(V(0.11,-0.85,0.05))}, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:kneeFwd, footDir:{L:V(0,-0.6,1),R:V(0,-0.6,1)}}); }}; }};
EX["Posture de l'enfant latérale"]={ cam:{target:V(0,0.3,0.2),dist:3.0,h:1.4,ang:60}, build(scene){
  floor(scene); const man=Man(scene);
  return { man, pose(t){ const pelvis=V(0,lerp(0.34,0.30,t),-0.20); const up=V(lerp(-0.1,-0.35,t),-0.28,1).normalize(), fwd=V(0,-1,-0.28).normalize();
    man.set({pelvis, up, fwd, bend:0.15, hand:{L:V(lerp(-0.30,-0.45,t),0.06,0.78),R:V(lerp(-0.12,-0.30,t),0.06,0.84)}, foot:{L:V(-0.12,0.07,-0.36),R:V(0.12,0.07,-0.36)},
      elbowPole:{L:V(0,1,0),R:V(0,1,0)}, kneePole:{L:V(0,-1,1),R:V(0,-1,1)}, footDir:{L:V(0,-0.2,-1),R:V(0,-0.2,-1)}}); }}; }};
EX["Étirement biceps au mur"]={ cam:{target:V(0.2,1.1,0),dist:3.2,h:1.5,ang:-30}, build(scene){
  floor(scene); const man=Man(scene); wall(scene,0.80,0,1.6,'x');
  return { man, pose(t){ const rot=lerp(0,25,t)*D; const fwd=V(-Math.sin(rot),0,Math.cos(rot));
    man.set({pelvis:V(0,0.93,0), up:V(0,1,0), fwd, hand:{L:V(-0.24,0.92,0.06),R:V(0.76,1.40,-0.18)}, foot:feetStand(0.14), elbowPole:{L:V(-1,0,0),R:V(0,-1,0)}, kneePole:kneeFwd, footDir:toesOut}); }}; }};
EX["Étirement avant-bras"]={ cam:{target:V(0.1,1.25,0.35),dist:2.2,h:1.45,ang:100}, build(scene){
  floor(scene); const man=Man(scene);
  return { man, pose(t){ const e=ease(t); const hr=V(0.13,1.36,0.60); const a=lerp(0,70,e)*D; // angle d'extension du poignet
    const dirR=V(0,-Math.sin(a),Math.cos(a)), nR=V(0,Math.cos(a),Math.sin(a)); // doigts : vers l'avant puis vers le bas ; paume vers le haut puis vers l'avant
    const tip=hr.clone().addScaledVector(dirR,0.09); const hl=tip.clone().addScaledVector(nR,0.035).add(V(-0.02,0,0)); // la main gauche appuie sur les doigts côté paume
    man.set({...STAND, hand:{L:hl,R:hr}, palm:{R:{dir:dirR,normal:nR},L:{dir:V(0.6,0,-0.2).addScaledVector(nR,-0.3),normal:nR.clone().negate()}}, foot:feetStand(0.14), elbowPole:{L:V(-1,-1,0),R:V(0.3,-1,0)}, kneePole:kneeFwd, footDir:toesOut}); }};
}};
// Chat-vache : mains et genoux fixes ; c'est la colonne qui s'arrondit (chat) ou se creuse (vache), la tête suit
EX["Chat-vache"]={ cam:{target:V(0,0.45,0.0),dist:3.0,h:1.05,ang:90}, build(scene){
  floor(scene); const man=Man(scene); const down=V(0,-1,0), eD=V(0,0.12,0.46).normalize(), n=V(0,0.46,-0.12).normalize(), Sc=V(0,0.62,0.24);
  return { man, pose(t){ const e=ease(t); // 0 = vache (dos creusé, tête haute), 1 = chat (dos rond, tête basse)
    const al=lerp(-22,25,e)*D; const P=Sc.clone().addScaledVector(eD,-0.5*Math.cos(al));
    const u=eD.clone().multiplyScalar(Math.cos(al)).addScaledVector(n,Math.sin(al)).normalize();
    man.set({pelvis:P, up:u, fwd:down, bend:2*al, hand:{L:V(-0.2,0.06,0.25),R:V(0.2,0.06,0.25)}, foot:{L:V(-0.12,0.07,P.z-0.46),R:V(0.12,0.07,P.z-0.46)}, elbowPole:{L:V(0,0,-1),R:V(0,0,-1)}, kneePole:{L:V(0,-1,0.3),R:V(0,-1,0.3)}, footDir:{L:V(0,-0.2,-1),R:V(0,-0.2,-1)}}); }};
}};
// Pigeon : jambe avant pliée en travers devant le bassin, jambe arrière tendue, buste qui se penche en avant
EX["Fente basse (fléchisseurs de hanche)"]={ cam:{target:V(0,0.6,0),dist:3.2,h:1.2,ang:80}, build(scene){
  floor(scene); const man=Man(scene);
  return { man, pose(t){ const pelvis=V(0,lerp(0.50,0.46,t),lerp(-0.08,0.04,t)); const C=pelvis.clone().add(V(0,0.5,0));
    man.set({pelvis, up:V(0,1,-0.05), fwd:V(0,0,1), hand:{L:V(-0.20,0.82,0.30),R:V(0.06,0.84,0.33)}, foot:{L:V(-0.12,0.07,0.42),R:V(0.13,0.07,-0.78)}, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(0,0,1),R:V(0,-1,0.3)}, footDir:{L:V(0,0,1),R:V(0,-0.1,-1)}}); }}; }};
EX["Étirement quadriceps debout"]={ cam:{target:V(0,1.0,0),dist:3.2,h:1.4,ang:70}, build(scene){
  floor(scene); const man=Man(scene); wall(scene,-0.72,0,1.6,'x');
  return { man, pose(t){ const f=V(0.11,lerp(0.70,0.76,t),lerp(-0.18,-0.22,t));
    man.set({...STAND, hand:{L:V(-0.66,1.30,0.05),R:f.clone().add(V(0.05,0.03,-0.04))}, foot:{L:V(-0.12,0.07,0.02),R:f}, elbowPole:{L:V(0,-1,0),R:V(1,0,0)}, kneePole:{L:V(-0.1,0,1),R:V(0,-1,0.15)}, footDir:{L:V(-0.1,0,1),R:V(0,1,-0.2)}}); }}; }};
EX["Étirement ischios sur banc"]={ cam:{target:V(0,0.8,0.2),dist:3.3,h:1.35,ang:80}, build(scene){
  floor(scene); const man=Man(scene); const b=box(0.32,0.07,0.6,MAT.pad); b.position.set(0.1,0.44,0.75); scene.add(b); fixed(scene,0.035,V(0.1,0.4,0.75),V(0.1,0,0.75));
  return { man, pose(t){ const T=tiltVecs(lerp(10,42,t)), pelvis=V(0,0.92,-0.08); const C=pelvis.clone().addScaledVector(T.up,0.5);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(0.02,lerp(0.80,0.66,t),lerp(0.36,0.48,t)),R:V(0.18,lerp(0.80,0.66,t),lerp(0.36,0.48,t))}, foot:{L:V(-0.12,0.07,-0.05),R:V(0.10,0.52,0.72)}, elbowPole:{L:V(-1,-0.5,0),R:V(1,-0.5,0)}, kneePole:{L:V(0,0,1),R:V(0,1,0.2)}, footDir:{L:V(-0.1,0,1),R:V(0,1,0.3)}}); }}; }};
EX["Pigeon"]={ cam:{target:V(0,0.3,0.0),dist:3.2,h:1.35,ang:110}, build(scene){
  floor(scene); const man=Man(scene);
  return { man, pose(t){ const e=ease(t); const T=tiltVecs(lerp(22,62,e)), pelvis=V(0.04,0.24,0.0);
    const C=pelvis.clone().addScaledVector(T.up,0.5);
    const hand={L:V(-0.30,0.06,lerp(0.30,0.72,e)),R:V(0.30,0.06,lerp(0.30,0.72,e))};
    man.set({pelvis, up:T.up, fwd:T.fwd, hand, foot:{L:V(-0.12,0.08,-0.82),R:V(-0.22,0.08,0.32)}, elbowPole:{L:V(-0.6,0,-1),R:V(0.6,0,-1)}, kneePole:{L:V(0,-1,0),R:V(1,-0.3,0.6)}, footDir:{L:V(0,-0.15,-1),R:V(-1,-0.1,0.1)}}); }};
}};
EX["Papillon"]={ cam:{target:V(0,0.3,0.1),dist:2.8,h:1.2,ang:25}, build(scene){
  floor(scene); const man=Man(scene);
  return { man, pose(t){ const T=tiltVecs(lerp(8,28,t)), pelvis=V(0,0.16,0.0);
    man.set({pelvis, up:T.up, fwd:T.fwd, hand:{L:V(-0.07,0.12,0.34),R:V(0.07,0.12,0.34)}, foot:{L:V(-0.06,0.08,0.36),R:V(0.06,0.08,0.36)}, elbowPole:{L:V(-1,0,0),R:V(1,0,0)}, kneePole:{L:V(-1,lerp(0.4,0.2,t),0.1),R:V(1,lerp(0.4,0.2,t),0.1)}, footDir:{L:V(1,0,0.3),R:V(-1,0,0.3)}}); }}; }};
EX["Genou vers le mur (cheville)"]={ cam:{target:V(0,0.7,0.2),dist:3.2,h:1.25,ang:80}, build(scene){
  floor(scene); const man=Man(scene); wall(scene,0,0.78,1.6,'z');
  return { man, pose(t){ const pelvis=V(0,lerp(0.86,0.80,t),lerp(0.12,0.24,t));
    man.set({pelvis, up:V(0,1,0.04), fwd:V(0,0,1), hand:{L:V(-0.25,1.30,0.72),R:V(0.25,1.30,0.72)}, foot:{L:V(0.12,0.07,0.58).setX(-0.12),R:V(0.12,0.07,-0.15)}, elbowPole:{L:V(-1,-1,0),R:V(1,-1,0)}, kneePole:{L:V(0,0,1),R:V(0,0,1)}, footDir:{L:V(0,0,1),R:V(0,-0.3,1)}}); }}; }};

const ease=x=>x<=0?0:x>=1?1:x*x*(3-2*x);

const TW=1;
// Avant-bras : bras tendu devant, paume vers le haut ; l'autre main tire les doigts vers le bas (poignet en extension)

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
