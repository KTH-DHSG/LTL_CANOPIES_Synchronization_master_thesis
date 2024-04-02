from math import sqrt

from matplotlib import pyplot
import matplotlib.patches


matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

def norm(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

def line_cost(line):
    t1 = [(line[i+1][0]-line[i][0], line[i+1][1]-line[i][1])  for i in xrange(0,len(line)-1)]
    T1 = [norm(n, [0,0]) for n in t1]
    return sum(T1)

def visualize_lines(plan1, plan2, seg1, seg2):
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    #===plan1
    X = [p[0] for p in plan1]
    Y = [p[1] for p in plan1]
    ax.plot(X, Y, 'ro-', markersize=8, linewidth=3, label=r'$\tau_{\mathcal{G}}^{a_i}$')
    #===plan2
    X = [p[0] for p in plan2]
    Y = [p[1] for p in plan2]
    ax.plot(X, Y, 'bs-', markersize=8, linewidth=3, label=r'$\tau_{\mathcal{G}}^{a_j}$')
    #===seg1
    X = [p[0] for p in seg1]
    Y = [p[1] for p in seg1]
    ax.plot(X, Y, '*-',color='purple',linewidth=3, label=r'$\tau_{\mathcal{G}}^{a_i}[l_1:l_2]$')
    #===seg2
    X = [p[0] for p in seg2]
    Y = [p[1] for p in seg2]
    ax.plot(X, Y, '*-', color='purple', linewidth=3, label=r'$\tau_{\mathcal{G}}^{a_j}[g_1:g_2]$')    
    ax.grid()
    ax.set_xlabel('$x(m)$')
    ax.set_ylabel('$y(m)$')
    ax.set_xlim(-0.5, 10)
    ax.set_ylim(-0.5, 13)
    ax.legend(ncol=4,shadow=True)
    return fig

plan1 = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 10.0], [4.0, 10.0], [6.0, 10.0], [7.0, 0.0], [8.0, 0.0],]
plan2 = [[0.5, 9.0], [1.5, 9.0], [2.5, 9.0], [3.5, 9.0], [4.0, 1.0], [6.0, 2.0], [7.0, 8.0], [8.0, 11.0],]
# ----------
# plan1 cost 26.10, plan2 cost 22.50, sum 48.60
# ----------
# new plan1 cost 9.71, new plan2 cost 12.52, sum 22.22




# plan1 = [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 1.0], [4.0, 1.5], [5.0, 2.5], [6.0, 1.5], [7.0, 8.5], [7.5, 9.5], [7.9, 10.0], [8.0, 2.0], [9.0, 1.0],[10.0, 3.0], [12.0, 2.5], [14.0, 3.0], [15.0, 1.0]]
# plan2 = [[0.0, 10.0], [0.5, 9.0], [1.5, 1.0], [2.5, 2.0], [3.5, 8.0], [4.0, 9.0], [6.0, 8.0], [7.0, 10.0], [8.0, 9.0],[10.0, 10.0], [12.0, 8.0], [14.0, 9.0]]
# ----------no time penalty
# ----------
# plan1 cost 35.03, plan2 cost 30.98, sum 66.01
# ----------
# new plan1 cost 29.64, new plan2 cost 28.53, sum 58.17
# ----------middle
# plan1 cost 35.03, plan2 cost 30.98, sum 66.01
# ----------
# new plan1 cost 34.36, new plan2 cost 29.13, sum 63.49
# ----------serve
# plan1 cost 35.03, plan2 cost 30.98, sum 66.01
# ----------
# new plan1 cost 32.71, new plan2 cost 32.85, sum 65.56



figure = visualize_lines(plan1, plan2, [], [])
print '----------'
print 'plan1 cost %.2f, plan2 cost %.2f, sum %.2f' %(line_cost(plan1), line_cost(plan2), line_cost(plan1)+line_cost(plan2))
figure.savefig(r'before.pdf', bbox_inches = 'tight')

#new_plan1, new_plan2 = swap(plan1, plan2)
cost = 1000
for s1 in range(1, len(plan1)-2):
    for e1 in range(s1+1, len(plan1)-1):
        for s2 in range(1, len(plan2)-2):
            for e2 in range(s2+1, len(plan2)-1):
                d1 = norm(plan1[s1-1], plan1[s1]) -  norm(plan1[s1-1], plan2[s2])
                d2 = norm(plan1[e1], plan1[e1+1]) -  norm(plan2[e2], plan1[e1+1])
                d3 = norm(plan2[s2-1], plan2[s2]) -  norm(plan2[s2-1], plan1[s1])
                d4 = norm(plan2[e2], plan2[e2+1]) -  norm(plan1[e1], plan2[e2+1])
                dcost = d1+d3+d2+d4
                if dcost > 0:
                    a1 = plan1[0:(s1-1)]
                    t1 = [(a[0]-b[0], a[1]-b[1])  for a in plan1[1:(s1)] for b in (plan1[0:(s1-1)]+[plan2[s2]])]
                    T1 = [norm(n, [0,0]) for n in t1]
                    t2 = [(a[0]-b[0], a[1]-b[1])  for a in plan2[1:(s2)] for b in (plan2[0:(s2-1)]+[plan1[s1]])]
                    T2 = [norm(n, [0,0]) for n in t2]
                    Tdif1 = abs(sum(T1)-sum(T2))
                    t3 = [(a[0]-b[0], a[1]-b[1])  for a in (plan1[1:(s1)]+plan2[s2:(e2+1)]+[plan1[e1+1]]) for b in (plan1[0:(s1)]+plan2[s2:(e2+1)])]
                    T3 = [norm(n, [0,0]) for n in t3]
                    t4 = [(a[0]-b[0], a[1]-b[1])  for a in (plan2[1:(s2)]+plan1[s1:(e1+1)]+[plan2[e2+1]]) for b in (plan2[0:(s2)]+plan1[s1:(e1+1)])]
                    T4 = [norm(n, [0,0]) for n in t4]
                    Tdif2 = abs(sum(T3)-sum(T4))
                    Totalcost = -dcost + 0.1*(Tdif1 + Tdif2)
                    if Totalcost < cost:
                        cost = Totalcost
                        S1 = s1
                        E1 = e1
                        S2 = s2
                        E2 = e2
seg1 = plan1[S1:(E1+1)]
seg2 = plan2[S2:(E2+1)]
new_plan1 = plan1[0:S1] + seg2 + plan1[(E1+1):]
new_plan2 = plan2[0:S2] + seg1 + plan2[(E2+1):]
print '----------'
print 'new plan1 cost %.2f, new plan2 cost %.2f, sum %.2f' %(line_cost(new_plan1), line_cost(new_plan2),  line_cost(new_plan1)+ line_cost(new_plan2))
figure = visualize_lines(new_plan1, new_plan2, seg1, seg2)
figure.savefig(r'after.pdf', bbox_inches = 'tight')



