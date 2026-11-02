#include "blood_decal_core.h"
#include <vector>
#include <cstdio>
#include <random>
using namespace BloodCore;

static M4 RotX(float a, const V3& pivot){ // rotasi di sumbu X (bengkok maju) sekitar pivot
    M4 m; float c=cosf(a), s=sinf(a);
    m.right=V3(1,0,0); m.up=V3(0,c,s); m.at=V3(0,-s,c);
    V3 pv = pivot; V3 rp = XformDir(m,pv); m.pos = pv - rp; return m; }
static M4 RotZ(float a, const V3& pivot){ // twist sekitar sumbu Z (putar badan)
    M4 m; float c=cosf(a), s=sinf(a);
    m.right=V3(c,s,0); m.up=V3(-s,c,0); m.at=V3(0,0,1);
    V3 pv = pivot; V3 rp = XformDir(m,pv); m.pos = pv - rp; return m; }
static M4 Mul(const M4& A, const M4& B){ // row-vector: p*A*B
    M4 r; r.right=XformDir(B,A.right); r.up=XformDir(B,A.up); r.at=XformDir(B,A.at);
    r.pos=XformPoint(B,A.pos); return r; }

int main(int argc,char**argv){
    const int rings = argc>1?atoi(argv[1]):13, segs = argc>2?atoi(argv[2]):28; printf("=== mesh rings=%d segs=%d ===\n",rings,segs);
    // ---- mesh torso elips: a=0.2 (lebar), b=0.12 (dalam), tinggi 0..0.6
 const float A=0.20f, Bd=0.12f, H=0.6f;
    std::vector<V3> bind; std::vector<float> w; std::vector<uint32_t> idx; std::vector<uint16_t> tris;
    const float boneZ[3]={0.0f,0.30f,0.60f};
    for(int r=0;r<rings;++r){ float z=H*r/(rings-1);
        for(int k=0;k<segs;++k){ float th=6.2831853f*k/segs;
            bind.push_back(V3(A*cosf(th), Bd*sinf(th), z));
            // bobot linear antar tulang
            float wt[4]={0,0,0,0}; uint32_t ib[4]={0,0,0,0};
            if(z<=boneZ[1]){ float t=(z-boneZ[0])/(boneZ[1]-boneZ[0]); wt[0]=1-t; wt[1]=t; ib[0]=0; ib[1]=1; }
            else { float t=(z-boneZ[1])/(boneZ[2]-boneZ[1]); wt[0]=1-t; wt[1]=t; ib[0]=1; ib[1]=2; }
            for(int q=0;q<4;++q) w.push_back(wt[q]);
            idx.push_back(ib[0]|(ib[1]<<8)|(ib[2]<<16)|(ib[3]<<24));
        }}
    for(int r=0;r<rings-1;++r) for(int k=0;k<segs;++k){ int k2=(k+1)%segs;
        uint16_t a=r*segs+k,b=r*segs+k2,c=(r+1)*segs+k,d=(r+1)*segs+k2;
        tris.insert(tris.end(),{a,b,c, b,d,c}); }
    int nv=bind.size(), nt=tris.size()/3;

    M4 skinToBone[3]; for(int b=0;b<3;++b){ skinToBone[b]=M4(); skinToBone[b].pos=V3(0,0,-boneZ[b]); }
    M4 live[3];
    auto setPose=[&](float bend1,float bend2,float twist){
        M4 bind0; // pelvis diam
        M4 b0; b0.pos=V3(0,0,0); live[0]=b0;
        // spine: bengkok sekitar pivot pelvis(0,0,0) lalu twist
        M4 m1=Mul(Mul(RotX(bend1,V3(0,0,0)),RotZ(twist*0.5f,V3(0,0,0))), M4()); 
        M4 t1; t1.pos=V3(0,0,boneZ[1]); live[1]=Mul(t1,m1);
        M4 m2=Mul(Mul(RotX(bend1+bend2,V3(0,0,0)),RotZ(twist,V3(0,0,0))), M4());
        M4 t2; t2.pos=V3(0,0,boneZ[2]); live[2]=Mul(t2,m2);
    };
    auto view=[&](){ SkinView s; s.bindPos=bind.data(); s.weights=w.data(); s.indices=idx.data();
        s.skinToBone=skinToBone; s.boneMatrix=live; s.numBones=3; s.numVerts=nv; return s; };

    auto skinAll=[&](std::vector<V3>& out){ SkinView s=view(); out.resize(nv); for(int i=0;i<nv;++i) out[i]=SkinVertex(s,i); };

    // jarak titik ke permukaan mesh ter-skin (segitiga terdekat)
    auto distToMesh=[&](const V3& p, const std::vector<V3>& sk){ float best=1e9f;
        for(int t=0;t<nt;++t){ V3 cp=ClosestPointOnTriangle(p,sk[tris[t*3]],sk[tris[t*3+1]],sk[tris[t*3+2]]); best=std::min(best,Len(cp-p)); } return best; };

    // ===== pose A: berdiri lurus; tembakan kena di dada depan; titik hit di "bola tabrak" 9 cm di luar kulit
    setPose(0,0,0);
    std::vector<V3> skA; skinAll(skA);
    V3 skinTruth(0, Bd, 0.40f);            // titik kulit sebenarnya (depan)
    V3 hitPos = skinTruth + V3(0,0.09f,0.01f);   // AddBlood pos = di bola tabrak, 9 cm melayang
    V3 axisPt(0,0,0.40f);
    V3 out = Norm(hitPos-axisPt);
    BoneFilter flt; // semua tulang boleh
    SkinView sv=view();
    SurfaceHit sh=FindSkinSurface(sv,tris.data(),nt,skA.data(),flt,hitPos,out,0.20f,0.30f);
    printf("[T1] raycast ok=%d  jarak titik-kena->kulit sebenarnya = %.4f m (hit melayang 0.09)\n", sh.ok, Len(sh.point-skinTruth));
    MeshDecal dec; V3 flow(0,0,-1);
    bool built=BuildMeshDecal(dec,sv,tris.data(),nt,skA.data(),flt,sh.point,sh.normal,flow,axisPt,0.08f,bind.data());
    printf("[T1] decal built=%d tris=%d verts=%d\n",built,dec.triCount,dec.vertCount);

    // ===== CARA LAMA: anchor di frame root (pos tetap relatif root, root tidak ikut bengkok) =====
    V3 oldAnchorRoot = hitPos;     // di frame root = dunia saat pose A (root diam)

    struct P{const char*n; float b1,b2,tw;} poses[]={{"lurus",0,0,0},{"bungkuk 25/25",0.44f,0.44f,0},{"twist 35deg (aim)",0,0,0.61f},{"bungkuk+twist",0.3f,0.5f,0.5f}};
    for(auto& ps:poses){
        setPose(ps.b1,ps.b2,ps.tw);
        std::vector<V3> sk; skinAll(sk);
        SkinView s2=view();
        std::vector<V3> pos(kDecalMaxVerts), nrm(kDecalMaxVerts);
        bool ok=EvalMeshDecal(dec,s2,0.003f,pos.data(),nrm.data());
        float maxGap=0,minGap=1e9f,sumGap=0;
        for(int i=0;i<dec.vertCount;++i){ float g=distToMesh(pos[i],sk); maxGap=std::max(maxGap,g); minGap=std::min(minGap,g); sumGap+=g; }
        // titik pusat decal = vertex terdekat dari sh.point? ambil rata2 posisi vertex untuk pusat
        // cara lama: titik anchor tetap di ruang dunia (root tidak berubah) -> jarak ke kulit:
        float oldGap=distToMesh(oldAnchorRoot,sk);
        // cara lama versi "snap ke elips root-frame": bandingkan titik kulit awal (skinTruth) yang tetap di dunia
        float oldGap2=distToMesh(skinTruth,sk);
        printf("[%-18s] T1 decal: ok=%d gap kulit min/avg/max = %.4f / %.4f / %.4f m (lift=0.003) | cara-lama(anchor hit)=%.3f m, cara-lama(anchor kulit pose awal)=%.3f m\n",
            ps.n,ok,minGap,sumGap/dec.vertCount,maxGap,oldGap,oldGap2);
    }

    // ===== TIER 2: sumbu poligon tulang + elips; parameter dibuat di pose A lalu dievaluasi di pose lain
    printf("\n--- Tier 2 (elips terikat poligon tulang belakang) ---\n");
    auto frameNow=[&](){ V3 pts[3]={live[0].pos, live[1].pos, live[2].pos};
        V3 rSh=XformPoint(live[2],V3(0.20f,0,0)), lSh=XformPoint(live[2],V3(-0.20f,0,0));
        return BuildTorsoFrame(pts,3,rSh,lSh); };
    setPose(0,0,0);
    TorsoFrame fA=frameNow();
    float s,th; TorsoParams(fA,hitPos,0.92f,s,th);
    V3 pA,nA; TorsoSurface(fA,s,th,0.92f,pA,nA);
    printf("params dari hit melayang: s=%.3f theta=%.1fdeg  titik elips di pose A -> kulit: %.4f m\n",s,th*57.2958f,distToMesh(pA,skA));
    for(auto& ps:poses){
        setPose(ps.b1,ps.b2,ps.tw);
        TorsoFrame f=frameNow();
        V3 P,N; TorsoSurface(f,s,th,0.92f,P,N);
        std::vector<V3> sk; skinAll(sk);
        printf("[%-18s] T2 titik elips -> kulit mesh: %.4f m (shrink 0.92 -> ideal ~0.01-0.02)\n",ps.n,distToMesh(P,sk));
    }
    return 0;
}
