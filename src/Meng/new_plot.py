from math import sqrt
from matplotlib import pyplot
import matplotlib.patches
from matplotlib.backends.backend_pdf import PdfPages
from collections import defaultdict
import random


matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

act_replace={
    'r': '$r$',
    'c':'$c$',
    'hca':'$hca$',
    'hcb':'$hcb$',
    'p21': '$p_2^1$', 
    'd21': '$d_2^1$',
    'p22': '$p_2^2$',
    'd22': '$d_2^2$',
    'hp22': '$hp_2^2$',
    'hd22': '$hd_2^2$',
    'p23': '$p_2^3$',
    'd23': '$d_2^3$',
    'hpa23': '$hpa_2^3$',
    'hda23': '$hda_2^3$',        
    'hpb23': '$hpb_2^3$',
    'hdb23': '$hdb_2^3$',
    'p31': '$p_3^1$', 
    'd31': '$d_3^1$',
    'p32': '$p_3^2$',
    'd32': '$d_3^2$',
    'hp32': '$hp_3^2$',
    'hd32': '$hd_3^2$',
    'p33': '$p_3^3$',
    'd33': '$d_3^3$',
    'hpa33': '$hpa_3^3$',
    'hda33': '$hda_3^3$',        
    'hpb33': '$hpb_3^3$',
    'hdb33': '$hdb_3^3$',
    'p41': '$p_4^1$', 
    'd41': '$d_4^1$',
    'p42': '$p_4^2$',
    'd42': '$d_4^2$',
    'hp42': '$hp_4^2$',
    'hd42': '$hd_4^2$',
    'p43': '$p_4^3$',
    'd43': '$d_4^3$',
    'hpa43': '$hpa_4^3$',
    'hda43': '$hda_4^3$',        
    'hpb43': '$hpb_4^3$',
    'hdb43': '$hdb_4^3$',    
}



def plot_AA3(AA, STEP, local, dep):
    global act_replace
    fig = pyplot.figure()
    ax= fig.add_subplot(111)
    marker_style = ['bs-', 'r*-', 'go-']
    label_style = ['$Local$', '$Assisting$', '$Collaborative$']
    T = len(AA)    
    Act = dict()
    l = [0, 0, 0] 
    for t in xrange(0,T):
        for aa in AA[t]:
            aa = tuple(aa)
            if (aa in Act) and (any(p[0]<=t<=p[1] for p in Act[aa])):
                continue
            else:
                if aa not in Act:
                    Act[aa] = []
                k=0
                while (((t+k) <= (len(AA)-1)) and (list(aa) in AA[t+k])):
                    k = k+1                
                Act[aa].append([t, t+k])
    #print Act
    for key, value in Act.iteritems():
        if key[1] in local:
            marker = marker_style[0]
            label = label_style[0]
            l[0] += 1
        elif key[1] in dep:
            marker = marker_style[2]
            label = label_style[2]
            l[2] += 1
        else:
            marker = marker_style[1]
            label = label_style[1]
            l[1] += 1
        for p in value:
            if (l[label_style.index(label)]==1):
                ax.plot([p[0]*STEP, p[1]*STEP], [key[0]+1, key[0]+1], marker, markersize=4, label=label, linewidth = 3)       
            else:
                ax.plot([p[0]*STEP, p[1]*STEP], [key[0]+1, key[0]+1], marker, markersize=4, linewidth = 3)                
            ax.text((p[0]*0.5+p[1]*0.5)*STEP, key[0]+1.2, r'$%s$' %act_replace[key[1]], fontsize=13, fontweight = 'bold')
    ax.set_xlabel('$Time (s)$')
    ax.set_ylabel('$Agent$')
    ax.set_xlim(0, (T+3)*STEP)
    ax.set_ylim(0, 24)
    ax.grid("on")
    ax.set_aspect(3)
    print ('actions:', l)
    ax.legend(ncol=3, shadow=True)
    pyplot.savefig('action2.pdf',bbox_inches='tight')


def plot_AA5(AA, STEP, local, dep):
    global act_replace
    fig = pyplot.figure()
    ax= fig.add_subplot(111)
    marker_style = ['bs-', 'r*-', 'go-']
    label_style = ['$Local$', '$Assisting$', '$Collaborative$']
    T = len(AA)    
    Act = dict()
    l = [0, 0, 0] 
    for t in xrange(0,T):
        for aa in AA[t]:
            aa = tuple(aa)
            if (aa in Act) and (any(p[0]<=t<=p[1] for p in Act[aa])):
                continue
            else:
                if aa not in Act:
                    Act[aa] = []
                k=0
                while (((t+k) <= (len(AA)-1)) and (list(aa) in AA[t+k])):
                    k = k+1                
                Act[aa].append([t, t+k])
    #print Act
    for key, value in Act.iteritems():
        if key[1] in local:
            marker = marker_style[0]
            label = label_style[0]
            l[0] += 1
        elif key[1] in dep:
            marker = marker_style[2]
            label = label_style[2]
            l[2] += 1
        else:
            marker = marker_style[1]
            label = label_style[1]
            l[1] += 1
        for p in value:
            if (l[label_style.index(label)]==1):
                ax.plot([p[0]*STEP, p[1]*STEP], [key[0]+1, key[0]+1], marker, markersize=4, label=label, linewidth = 3)       
            else:
                ax.plot([p[0]*STEP, p[1]*STEP], [key[0]+1, key[0]+1], marker, markersize=4, linewidth = 3)                
            ax.text((p[0]*0.5+p[1]*0.5)*STEP, key[0]+1.2, r'$%s$' %act_replace[key[1]], fontsize=13, fontweight = 'bold')
    ax.set_xlabel('$Time (s)$')
    ax.set_ylabel('$Agent$')
    ax.set_xlim(0, (T+3)*STEP)
    ax.set_ylim(0, 24)
    ax.grid("on")
    ax.set_aspect(5)
    print ('actions:', l)
    ax.legend(ncol=3, shadow=True)
    pyplot.savefig('action3.pdf',bbox_inches='tight')
