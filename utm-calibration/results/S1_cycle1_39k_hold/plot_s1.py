import csv, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
r=list(csv.DictReader(open("S1_cycle1_CAL.csv")))
F=np.array([float(x["utm_N"]) for x in r]); V=np.array([float(x["mean_mV"]) for x in r])
G=np.array([float(x["g_uS"]) for x in r]); SD=np.array([float(x["sd_mV"]) for x in r]); base=float(r[0]["base_mV"])
L=F>0
s,c=np.polyfit(F,V,1); resN=(V-(s*F+c))/s
b,la=np.polyfit(np.log(G[L]),np.log(F[L]),1); a=np.exp(la)
for _ in range(50):
    p=a*G[L]**b; J=np.vstack([G[L]**b,a*G[L]**b*np.log(G[L])]).T; d=np.linalg.lstsq(J,F[L]-p,rcond=None)[0]; a+=d[0]; b+=d[1]
ep=a*G[L]**b-F[L]
print(f"line V = {s:.4f} mV/N * F + {c:.1f}, max dev {abs(resN).max():.1f} N, RMSE {np.sqrt((resN**2).mean()):.1f} N")
print(f"power F = {a:.4f} * G^{b:.4f}, RMSE {np.sqrt((ep**2).mean()):.1f} N, max {abs(ep).max():.1f} N")
INK,MUT,GRID,C1,C2="#0b0b0b","#52514e","#e4e3df","#2a78d6","#eb6834"
plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False,"axes.grid":True,"grid.color":GRID,"axes.edgecolor":MUT,"legend.frameon":False})
fig,ax=plt.subplots(1,2,figsize=(12,5),dpi=150,gridspec_kw=dict(width_ratios=[1.6,1]))
A=ax[0]
A.axhspan(3250,3400,color="#f2f1ee",zorder=0); A.text(10,3265,"ADC clip > 3250 mV",color=MUT,fontsize=8)
ff=np.linspace(0,1000,200)
A.plot(ff,s*ff+c,"--",color=MUT,lw=1,label=f"straight line: {s:.3f} mV/N, +{c:.0f} mV")
gg=np.linspace(0.5,G.max()*1.03,200)
A.plot(a*gg**b,base*(1+39000*gg*1e-6),"-",color=C2,lw=1.2,label=f"power fit F = {a:.3f}·G^{b:.3f} (RMSE {np.sqrt((ep**2).mean()):.1f} N)")
A.errorbar(F,V,yerr=SD,fmt="o",color=C1,ms=6,capsize=2,label="hold mean (last 2 s), loading")
for f,v in zip(F,V): A.annotate(f"{v:.0f}",(f,v),textcoords="offset points",xytext=(-6,8),fontsize=7,color=MUT,ha="right")
A.set_xlabel("UTM force (N)"); A.set_ylabel("FSR output (mV)"); A.set_xlim(-20,1050); A.set_ylim(0,3400)
A.set_title("S1, RF 39 kΩ, rigid plate, 30 s holds, loading only (cycle 1)",loc="left",color=INK,fontsize=10)
A.legend(loc="upper left")
B=ax[1]
B.axhline(0,color=MUT,lw=1)
B.plot(F,resN,"o-",color=C1,ms=5,label="straight line")
B.plot(F[L],ep,"s-",color=C2,ms=4,label="power fit")
B.set_xlabel("UTM force (N)"); B.set_ylabel("Fit error (N)"); B.set_title("Error of each fit",loc="left",color=INK,fontsize=10); B.legend()
fig.tight_layout(); fig.savefig("S1_cycle1_mV_vs_N.png")
