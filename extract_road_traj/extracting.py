import sys, os

t2id = open(sys.argv[2] + 't2id.csv','w')
tid = 0
trajs = open(sys.argv[2] + 'trajs.txt','w')
temps = open(sys.argv[2] + 'temporals.txt', 'w')

for traj in os.listdir(sys.argv[1]):
    pname = sys.argv[1] + traj
    segs = []
    tps = []
    with open(pname) as fin:
        for row in fin.readlines():
            eid = row.split(',')[3]
            tp = row.split(',')[0]
            if len(segs)==0 or segs[-1] != eid:
                segs.append(eid)
                tps.append(tp)
    print >> t2id, str(tid)+','+traj
    print >> trajs, tid,len(segs), ' '.join(segs)
    print >> temps, tid,len(tps), ' '.join(tps)
    tid += 1
