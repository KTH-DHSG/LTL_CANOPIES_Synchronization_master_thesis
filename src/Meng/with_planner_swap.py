from math import sqrt
from matplotlib import pyplot
import matplotlib.patches
from matplotlib.backends.backend_pdf import PdfPages
from collections import defaultdict
import random

from agent_model_swap import *


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

act_replace2={
    'r': '$record$',
    'c':'$circle$',
    'hca':'$hcirclea$',
    'hcb':'$hcircleb$',
    'p21': '$pick_2^1$', 
    'd21': '$drop_2^1$',
    'p22': '$pick_2^2$',
    'd22': '$drop_2^2$',
    'hp22': '$hpick_2^2$',
    'hd22': '$hdrop_2^2$',
    'p23': '$pick_2^3$',
    'd23': '$drop_2^3$',
    'hpa23': '$hpicka_2^3$',
    'hda23': '$hdropa_2^3$',        
    'hpb23': '$hpickb_2^3$',
    'hdb23': '$hdropb_2^3$',
    'p31': '$pick_3^1$', 
    'd31': '$drop_3^1$',
    'p32': '$pick_3^2$',
    'd32': '$drop_3^2$',
    'hp32': '$hpick_3^2$',
    'hd32': '$hdrop_3^2$',
    'p33': '$pick_3^3$',
    'd33': '$drop_3^3$',
    'hpa33': '$hpicka_3^3$',
    'hda33': '$hdropa_3^3$',        
    'hpb33': '$hpickb_3^3$',
    'hdb33': '$hdropb_3^3$',
    'p41': '$pick_4^1$', 
    'd41': '$drop_4^1$',
    'p42': '$pick_4^2$',
    'd42': '$drop_4^2$',
    'hp42': '$hpick_4^2$',
    'hd42': '$hdrop_4^2$',
    'p43': '$pick_4^3$',
    'd43': '$drop_4^3$',
    'hpa43': '$hpicka_4^3$',
    'hda43': '$hdropa_4^3$',        
    'hpb43': '$hpickb_4^3$',
    'hdb43': '$hdropb_4^3$',    
}


def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001          
                
def reach(x, reg, radi):
    if ((abs(x[0]-reg[0])<= reg[2]*radi) 
        and (abs(x[1]-reg[1])<= reg[2]*radi)):
        return True
    return False

def movie_clips(R_dict, S, Obs, G,  XX=None, CC=None, AA=None, L=10, WS=None):
    K = range(0, len(XX[0]), L)
    for k in xrange(1,len(K)):  
        XX_hat = []  # for traj
        CC_hat = []  # for communication
        AA_hat = []  # for actions
        CC_hat.append(CC[k])
        AA_hat.append(AA[k])         
        for i in range(0, len(XX)):
            XX_hat.append(XX[i][k])            
        figure = visualize_traj(R_dict, S, Obs, G, XX_hat, CC_hat, AA_hat, None, WS)
        # if (actual_elements(AA_hat)>=2) or (actual_elements(CC_hat)>=4):
        #     figure.savefig('movie/frame%s.pdf' %k, bbox_inches='tight')        
        figure.savefig('movie/frame%s.png' %k, dpi = 100)

def actual_elements(AA):
    k = 0 
    for A in AA:
        for aa in A:
           if aa:
               k +=1
    return k
           

def visualize_traj(R_dict, S, Obs, G,  XX=None, CC=None, AA=None, name=None, WS_ts=None):
    global act_replace2
    # [(time, no_messages)]
    figure = pyplot.figure()
    axes = figure.add_subplot(1,1,1)
    #axes = figure.add_axes((0,0,.8,1))
    #==============================
    # draw R1--R6
    for R,label in R_dict.iteritems():
        circle = matplotlib.patches.Circle(
            (R[0], R[1]),
            radius = R[2],
            facecolor = 'y',
            fill = True,
            edgecolor = 'black',
            linewidth = 1,
            ls = 'solid',
            alpha = 0.6)
        axes.add_patch(circle)
        for l in label:
            if l == 'r1':
                l = 'R_1'
            if l == 'r2':
                l = 'R_2'
            if l == 'r3':
                l = 'R_3'
            if l == 'r4':
                l = 'R_4'
            if l == 'r5':
                l = 'R_5'
            if l == 'r6':
                l = 'R_6'                                                                
            axes.text(R[0]-0.2*R[2],R[1]-0.2*R[2],r'$%s$' %l,
                  fontsize=12, fontweight ='bold')        
    #==============================
    # draw S1--S3    
    for s in S:
        rect = matplotlib.patches.Rectangle(
            [s[0][0]-s[0][2],s[0][1]-s[0][2]],
            2*s[0][2], 2*s[0][2],
            facecolor='blue',
            fill = True,
            edgecolor='black',
            linewidth=1,
            ls='solid',
            alpha=0.6)
        axes.add_patch(rect)
        if s[1] == 's1':
            s[1] = 'S_1'
        if s[1] == 's2':
            s[1] = 'S_2'
        if s[1] == 's3':
            s[1] = 'S_3'
        axes.text(s[0][0]-0.2*s[0][2],s[0][1]-0.2*s[0][2],r'$%s$' %s[1],
              fontsize=12, fontweight ='bold')
    #==============================
    # draw obstacle
    for obs in Obs:
        rects = matplotlib.patches.Rectangle(
            [obs[0]-obs[2],obs[1]-obs[2]],
            2*obs[2], 2*obs[2],
            facecolor='red',
            fill = True,
            edgecolor='black',
            linewidth=1,
            ls='solid',
            alpha=0.8)
        axes.add_patch(rects)
    #==============================
    # draw starting points
    g_color = ['green', 'cyan', 'purple', '#FF00FF']
    g_marker = ['o', 's', '^', '*'] 
    k = 0
    for g in G:
        rec = matplotlib.patches.Rectangle(
            [g[0][0]-g[0][2],g[0][1]-g[0][2]],
            2*g[0][2], 2*g[0][2],
            facecolor= g_color[k],
            fill = True,
            edgecolor='black',
            linewidth=1,
            ls='solid',
            alpha=0.8)
        axes.add_patch(rec)
        if g[1] == 'g1':
            g[1] = 'g_1'
        if g[1] == 'g2':
            g[1] = 'g_2'
        if g[1] == 'g3':
            g[1] = 'g_3'
        if g[1] == 'g4':
            g[1] = 'g_4'                    
        axes.text(g[0][0]-0.2*g[0][2],g[0][1]-0.2*g[0][2],r'$%s$' %g[1],
              fontsize=12, fontweight ='bold')
        k += 1
    # draw WS_ts
    if WS_ts:
        for (n1, n2) in WS_ts.edges():
            edge = matplotlib.lines.Line2D(
            [n1[0], n2[0]], [n1[1], n2[1]], 
            linestyle='-',
            linewidth=1.0,
            color='blue')
        axes.add_line(edge)                        
    ########### plot traj
    if XX:
        for i in range(0,len(XX)):
            if 0 <= i <=4:
                k = 0
            if 5 <= i <=9:
                k = 1
            if 10 <= i <= 14:
                k = 2
            if 15 <= i <= 19 :
                k = 3                                
            srec = matplotlib.patches.Rectangle(
                XX[i],
                2*0.5, 2*0.5,
                facecolor= g_color[k],
                fill = True,
                edgecolor='black',
                linewidth=1,
                ls='solid',
                alpha=1)
            axes.add_patch(srec)            
    ############ plot actions
    if AA:
        for A in AA:
            for aa in A:
                pose = XX[aa[0]]
                shiftx = 5
                shifty = 3
                if 0 <= aa[0] <=4:
                    k = 0
                if 5 <= aa[0] <=9:
                    k = 1
                if 10 <= aa[0] <= 14:
                    k = 2
                if 15 <= aa[0] <= 19 :
                    k = 3                                                
                if  k == 0:
                    t= axes.text(pose[0]-shiftx,pose[1],r'$%s$' %act_replace2[aa[1]],
                                 fontsize=18, fontweight ='bold')
                if k==1:
                    t= axes.text(pose[0],pose[1],r'$%s$' %act_replace2[aa[1]],
                                 fontsize=18, fontweight ='bold')
                if k==2:
                    t= axes.text(pose[0],pose[1]-shifty,r'$%s$' %act_replace2[aa[1]],
                                 fontsize=18, fontweight ='bold')
                if k==3:
                    t= axes.text(pose[0]-shiftx,pose[1]-shifty,r'$%s$' %act_replace2[aa[1]],
                                 fontsize=18, fontweight ='bold')
                t.set_bbox(dict(color=g_color[k], alpha=0.7, edgecolor=g_color[k]))                    
    ########### plot communication
    if CC:
        for C in CC:
            for cc in C:
                pose1 = XX[cc[0]]
                pose2 = XX[cc[1]]
                line = matplotlib.lines.Line2D(
                    [pose1[0]+0.5, pose2[0]+0.5], [pose1[1]+0.5, pose2[1]+0.5], 
                    linestyle='-',
                    linewidth=1.2,
                    color='black')
                axes.add_line(line)
    ########## labels and axes
    axes.set_aspect('equal')
    axes.set_xlim(0, 60)
    axes.set_ylim(0, 60)
    #axes.set_axis_off()
    axes.set_xlabel('$y (m)$')
    axes.set_ylabel('$x (m)$')
    # remove duplicate legends
    # handles, labels = axes.get_legend_handles_labels()
    # axes.legend([handle for i,handle in enumerate(handles) if i in display],
    #   [label for i,label in enumerate(labels) if i in display], loc = 'best')
    axes.grid("on")
    if name:
        pyplot.savefig('%s.pdf' %name,bbox_inches='tight')
    return figure
###################################################
#####################################################

def plot_CC(CC, STEP):
    T = len(CC)
    N = []
    t = []
    for i, C in enumerate(CC):
        if len(C):
            t.extend([i*STEP])
            if len(C) > 20:
                N.extend([20])
            else:
                N.extend([len(C)])
    fig = pyplot.figure()
    ax= fig.add_subplot(111)
    ax.bar(t, N, STEP)
    ax.set_xlabel('$Time (s)$')
    ax.set_ylabel('$Messages$')
    ax.set_xlim(0, (T+3)*STEP)
    ax.set_ylim(0, max(N)+1)
    ax.grid("on")
    ax.set_aspect('equal')
    pyplot.savefig('com_swap.pdf',bbox_inches='tight')


def plot_AA(AA, STEP, local, dep):
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
    ax.set_aspect('auto')
    print 'actions:', l
    ax.legend(ncol=3, shadow=True)
    pyplot.savefig('action_swap.pdf',bbox_inches='tight')

############################################
STEP = 0.1
t=0
tt=0
N = 20
f = [0,]*N # for recording plan progress
K=[0,]*N # for counting down action time
T=[0,]*N # to count the consecutive time of requesting
horizon = [5,]*N
ACT = ['action',]*N
DELAY = 10
T_bound = 200
discrete_plan = [0,]*N # [[k, plan], [k,plan]]
com_radius= 20
neighbor = dict()

visualize_traj(R_dict, S, Obs, G, XX=None, CC=None, AA=None, name='workspace', WS_ts = None)    
###################
# initialize 
Planner = [0,]*N
Planner[0] = a11_planner
Planner[1] = a12_planner
Planner[2] = a13_planner
Planner[3] = a14_planner
Planner[4] = a15_planner
Planner[5] = a21_planner
Planner[6] = a22_planner
Planner[7] = a23_planner
Planner[8] = a24_planner
Planner[9] = a25_planner
Planner[10] = a31_planner
Planner[11] = a32_planner
Planner[12] = a33_planner
Planner[13] = a34_planner
Planner[14] = a35_planner
Planner[15] = a41_planner
Planner[16] = a42_planner
Planner[17] = a43_planner
Planner[18] = a44_planner
Planner[19] = a45_planner

LEADER = {0: [1,2,3,4],
          5: [6,7,8,9],
          10: [11, 12, 13, 14],
          15: [16, 17, 18, 19]}
#com_counter = 0
X = list(init_pose)
C = []
A = []
CC = list()
AA = list()
global_com_bound = 5


##################### initial plan synthesis
for i in xrange(0, N):
    print 'Agent A%s' %(i)
    Planner[i].optimal(segment='prefix')
    #Planner[i].optimal()



##################### running time
while(tt<T_bound):
#while(True):
    global_com_bound -= STEP
    Request = dict()
    Reply = dict()
    C = []
    A = []
    neighbor = dict()
    print 'Time %s:' %tt
    #print 'agent positions', X
    for i in xrange(0, N):
        Planner[i].pose = X[i]
        Planner[i].traj.append(list(X[i]))
        todo = Planner[i].next_move
        #======================
        if not isinstance(todo, basestring):
            # motions
            reg = todo[:]
            if not reach(X[i], reg, 0.6):
                if not discrete_plan[i]:
                    discrete_plan[i] = route_ws(WS_ts, X[i], reg)
                waypoint = discrete_plan[i][1][discrete_plan[i][0]]
                ##### motion 
                u1 = (SPEED[i]*(waypoint[0]-X[i][0])/distance(waypoint, X[i]) 
                      )
                u2 = (SPEED[i]*(waypoint[1]-X[i][1])/distance(waypoint, X[i])
                      )
                X[i][0] += u1*STEP
                X[i][1] += u2*STEP
                print ('agent%s is moving towards %s\n' %(str(i), str(waypoint)))
                print ('agent%s at %s\n' %(str(i), str(X[i])))
                if reach(X[i], waypoint, 0.5):
                    if discrete_plan[i][0] <= len(discrete_plan[i][1])-2:
                        discrete_plan[i][0] += 1
                    else:
                        discrete_plan[i] = []
                if Planner[i].contract_time>0:
                    Planner[i].contract_time -= STEP
            else:
                if Planner[i].segment == 'loop':
                    if f[i] == 0:
                        print ('agent%s finished its task\n' 
                           %(str(i+1)))
                        f[i] =1
                Planner[i].find_next_move()
        else:
            # actions
            act = todo[:]
            if act in LOCAL:
                ############################### local action
                if K[i]<=0:
                    if act != ACT[i]:
                        # new local action
                        print ('agent%s start doing local action %s\n' 
                           %(str(i+1), str(act)))
                        K[i] = LOCAL[act]/STEP
                        ACT[i] = act
                    else:
                        # current action done
                        print ('agent%s: action %s done at %s s\n' 
                           %(str(i+1), str(act), str(tt)))
                        Planner[i].find_next_move()
                        if Planner[i].segment == 'loop':
                            if f[i] == 0:
                                print ('agent%s finished its task\n' 
                                       %(str(i+1)))
                                f[i] = 1
                else:
                    K[i] -= 1
                    # new local action
                    if act != 'None':
                        print ('agent%s is doing local action %s\n' 
                               %(str(i+1), str(act)))
                        A.append([i, str(act)])
                    if Planner[i].contract_time>0:
                        Planner[i].contract_time -= STEP
                    u1 = 0
                    u2 = 0
                    X[i][0] += u1*STEP
                    X[i][1] += u2*STEP 
            else:
                ######################### cooperative or assisting actions  
                if K[i]<=0:
                    if act != ACT[i]:
                        # new action
                        if act in dep:
                            print ('agent%s start doing cooperative action %s\n' 
                               %(str(i+1), str(act)))
                        else:
                            print ('agent%s start doing assisting action %s\n' 
                               %(str(i+1), str(act)))
                            print Planner[i].contract_time
                        if Planner[i].contract_time>0:
                            K[i] = int(round(Planner[i].contract_time/STEP))
                            ACT[i] = act
                        else:
                            print Planner[i].contract_time
                            print 'before the contract_time finishes, it has to be delayed'
                            # delay the cooperation by wait
                            Planner[i].delay_cooperation(DELAY, SPEED[i])
                    else:
                        # current action done
                        #print Planner[i].contract_time
                        print '********************************'
                        print '********************************'
                        print ('agent%s: action %s done at %s s\n' 
                           %(str(i+1), str(act), str(tt)))
                        print '********************************'
                        print '********************************'
                        if Planner[i].segment == 'loop':
                            if f[i] == 0:
                                print ('agent%s finished its task\n'  %(str(i+1)))
                                f[i] = 1
                        Planner[i].find_next_move()
                else:
                    K[i] -= 1
                    print ('agent%s is doing action %s\n' 
                       %(str(i+1), str(act)))
                    A.append([i, str(act)])                                  
                    if Planner[i].contract_time>0:
                        Planner[i].contract_time -= STEP
                    u1 = STEP*random.gauss(0,STEP)
                    u2 = STEP*random.gauss(0,STEP)
                    X[i][0] += u1*STEP
                    X[i][1] += u2*STEP 
        #======================
        # cooperation request
        if (T[i]<=-10):
            #print 'Planner[i].contract_time', Planner[i].contract_time
            #print 'T[i]', T[i]
            Request[i] = Planner[i].cooperative_action_in_horizon(dep,
                                                      horizon[i])
        T[i] -= 1
        # determine neighbors
        neighbor[i] = []      
        for j in xrange(0,N):
            if ((j != i) and (distance(X[i], X[j]) < com_radius)):
                neighbor[i].extend([j])                    
    #print Request
    #======================
    # choose one agent to serve
    agents_req = [a for a,b in Request.iteritems() if b]
    if agents_req:
        print '**************'
        print 'agents requested at time %s:' %tt, agents_req
        choosen_agent = random.choice(agents_req)
        #choosen_agent = agents_req[0]
        request = Request[choosen_agent]
        print 'choosen', choosen_agent, 'on', request
        print '**************'
        Reply = dict()
        for i in neighbor[choosen_agent]:
            # second layer communication 
            print 'local coordination'
            if (i!=choosen_agent):
                Reply[i] = Planner[i].evaluate_request(request, alpha=1)
                #print 'reply from agent %s' %str(i+1) , Reply[i]
                C.append([choosen_agent, i])
                #com_counter = 3
        Confirm, time = Planner[choosen_agent].confirmation(request, 
            Reply)
        print 'Confirmation',Confirm
        #print 'choosen_agent contract time', Planner[choosen_agent].contract_time
        #print '**************'
        if time>0:
            print 'local coordination successful'
            for i in Reply.keys():
                if (i!=choosen_agent):
                    Planner[i].adapt_plan(Confirm[i])
                    T[i] = int(round(Planner[i].contract_time/STEP))
            T[choosen_agent] = int(round(Planner[choosen_agent].contract_time/STEP))    
        else:
            # rely on first layer communication
            if  global_com_bound < 0:
                Reply = dict()          
                for i in LEADER.keys():
                    # first layer communication 
                    print 'global communication'
                    if (i!=choosen_agent):
                        Reply[i] = Planner[i].evaluate_request(request, alpha=1)
                        #print 'reply from agent %s' %str(i+1) , Reply[i]
                        C.append([choosen_agent, i])
                        #com_counter = 3
                    for j in LEADER[i]:
                        if (j!=choosen_agent):
                            Reply[j] = Planner[j].evaluate_request(request, alpha=1)
                            #print 'reply from agent %s' %str(i+1) , Reply[i]
                            C.append([i, j])
                global_com_bound = random.randint(3,8) 
                Confirm = dict()                  
                Confirm, time = Planner[choosen_agent].confirmation(request, Reply)
                print 'Confirmation', Confirm
                #print 'choosen_agent contract time', Planner[choosen_agent].contract_time
                #print '**************'
                if time>0:
                    print 'golobal coordination successful'
                    for i in Reply.keys():
                        if (i!=choosen_agent):
                            Planner[i].adapt_plan(Confirm[i])
                            T[i] = int(round(Planner[i].contract_time/STEP))
                    T[choosen_agent] = int(round(Planner[choosen_agent].contract_time/STEP))    
                else:                    
                    T[choosen_agent] = int(round(DELAY/STEP))
    #print 'T', T
    t=t+1
    tt = t*STEP
    # if com_counter == 3:
    #     last_C = C
    # if com_counter >=1:
    #     C = last_C
    # else:
    #     last_C = []
    #     C = []
    CC.append(C)    
    AA.append(A)
    if all(item>0 for item in f):
        break



XX = list()
for i in xrange(0,N):
    XX.append(Planner[i].traj)
#figure = visualize_traj(R_dict, S, Obs, G, XX, CC, AA, 'test', WS=None)
movie_clips(R_dict, S, Obs, G,  XX, CC, AA, L=3, WS=None)
plot_CC(CC, STEP)
plot_AA(AA, STEP, LOCAL, dep)
