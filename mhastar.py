# Joseph Reeves, Gen AI course project
# Implementation of IHMA* SMHA* on NQueens problem
# algorithms from paper Multi-Heuristic A*, Sandip Aine, et al., ijrr (2015)

# Run e.g.,
# > python3 nqueens.py -n 8 -t 0

import sys
import getopt
from heapq import heappush, heappop

class bcolors:
    OKRED = '\033[91m'
    ENDC = '\033[0m'

# PQNode and PQ from Gen AI Hw 1
class PQNode:
    
    def __init__(self, state, path, cost_from_start):
        self.state = state
        self.path = path
        self.g = cost_from_start

    def __gt__(self, other_node):
        return self.state.nq > other_node.state.nq # problem specific implementation!!

class PriorityQueue:

    def __init__(self):
        self.elements = []
        
    def top(self): # Questionable - elements may not be a heap
      (k,s) = heappop(self.elements)
      heappush(self.elements, (k,s))
      return (k,s)
    
    def nonempty(self):
        return bool(self.elements)

    def push(self, element, priority):
        heappush(self.elements, (priority, element))
        
    def push_update(self,element,priority):
        fnd = [(p,e) for p,e in self.elements if e.state==element.state]
        if len(fnd) > 0: self.elements.remove((fnd[0][0],fnd[0][1]))
        self.push(element,priority)

    def pop(self):
        return heappop(self.elements)[1]
    
    def contains(self, state):
        return any(
            element.state == state
            for priority, element in self.elements
        )

class imha:
  hs = []
  nh = 0
  h0 = None
  p = None #planner with (start, isGoal, getSucc)
  h0q = None
  hqs = None
  w1 = 0
  w2 = 0
  closed = []
  closed0 = []
  cnt = 0
  
  def __init__(self, planner, anchor, heuritics,w1,w2):
    self.p  = planner
    self.h0 = anchor
    self.nh = len(heuritics)
    self.hs = heuritics
    self.w1 = w1
    self.w2 = w2
    
  def key(self,g,h):
    return g + self.w1 * h
    
  def expand(self,s,i):
    succ = self.p.succ(s.state)
    closed = []
    heap = []
    g_s = s.g
    p = s.path
    h = None
    if i == -1: # Anchor
      closed = self.closed0
      heap = self.h0q
      h = self.h0
    else:
      closed = self.closed[i]
      heap = self.hqs[i]
      h = self.hs[i]
    for (newS,action) in succ:
      insert = False
      if newS in closed: continue
      # Cost meaningless for N-Queens so this part of alg ignored...
      # i.e., can never reach a state with shorter cost g, always exact same cost
      # and only one way to reach it...
#      exist_s = heap.find(newS)
#      if not exist_s: #insert = True
#      if insert or exist_s.g > g_s + self.p.cost(s,newS):
#        heap.remove(newS)
      g_newS = g_s + self.p.cost(s,newS) #cost + g_s
      key_newS = self.key(g_s+1,h(newS))
      p_newS = p[:] + [action]
      heap.push(PQNode(newS,p_newS,g_newS),key_newS)
        
  def search(self,start):
    #initialize Queues
    self.h0q = PriorityQueue()
    self.h0q.push(PQNode(start, [], 0),self.key(0,self.h0(start)))
    self.hqs = []
    for i in range(self.nh):
      self.closed.append([])
      self.hqs.append(PriorityQueue())
      self.hqs[i].push(PQNode(start, [], 0),self.key(0,self.hs[i](start)))

    while self.h0q.nonempty: #no check for infinity in N-Queens space
      self.cnt+=1
      for i in range(self.nh):
        (k,n) = self.hqs[i].top()
        (k0,n0) = self.h0q.top()
        if k <= self.w2 * k0:
          if self.p.is_goal(n.state): # open,i has path to goal
            print(i)
            return n.path
          else: # Expand open,i
            s = self.hqs[i].pop()
            self.expand(n,i)
            self.closed[i].append(n.state)
        else:
          if self.p.is_goal(n0.state): # anchor has path to goal
            print('Anchor')
            return n0.path
          else: # Expand Anchor
#            self.cnt+=1
            s = self.h0q.pop()
            self.expand(n0,-1)
            self.closed0.append(n0.state)


class smha:
  hs = []
  nh = 0
  h0 = None
  p = None #planner with (start, isGoal, getSucc)
  h0q = None
  hqs = None
  w1 = 0
  w2 = 0
  closedi = []
  closed0 = []
  cnt = 0
  g = []
#  g0 ={} # g-values in lookup dictionary, state -> g-value
#  gis = {}
  
  def __init__(self, planner, anchor, heuritics,w1,w2):
    self.p  = planner
    self.h0 = anchor
    self.nh = len(heuritics)
    self.hs = heuritics
    self.w1 = w1
    self.w2 = w2
    
  def key(self,g,h):
    return g + self.w1 * h
    
  def expand(self,s):
    succ = self.p.succ(s.state)
    g_s = s.g
    p = s.path
    for (newS,action) in succ:
      p_newS = p[:] + [action]
      if p_newS not in self.g:
        self.g.append(p_newS)
        if not newS in self.closed0:
          g_newS = g_s + self.p.cost(s,newS)
          key_newS = self.key(g_s+1,self.h0(newS))
          p_newS = p[:] + [action]
          self.h0q.push_update(PQNode(newS,p_newS,g_newS),key_newS)
          if not newS in self.closedi:
            for i in range(self.nh):
              if self.key(g_s+1,self.hs[i](newS)) <= self.key(g_s+1,self.h0(newS)) :
                key_newS = self.key(g_s+1,self.hs[i](newS))
                p_newS = p[:] + [action]
                self.hqs[i].push_update(PQNode(newS,p_newS,g_newS),key_newS)
        
  def search(self,start):
    #initialize Queues
    self.h0q = PriorityQueue()
    self.h0q.push(PQNode(start, [], 0),self.key(0,self.h0(start)))
    self.hqs = []
    for i in range(self.nh):
      self.hqs.append(PriorityQueue())
      self.hqs[i].push(PQNode(start, [], 0),self.key(0,self.hs[i](start)))
    
    while self.h0q.nonempty: #no check for infinity in N-Queens space
      self.cnt+=1
      if self.cnt > 9000: print("AHHH")
      for i in range(self.nh):
        (k,n) = self.hqs[i].top()
        (k0,n0) = self.h0q.top()
        if k <= self.w2 * k0:
          if self.p.is_goal(n.state): # open,i has path to goal
            return n.path
          else: # expand open,i
            s = self.hqs[i].pop()
            self.expand(n)
            self.closedi.append(n.state)
        else:
          if self.p.is_goal(n0.state): # anchor has path to goal
            return n0.path
          else: # expand anchor
#            self.cnt+=1
            s = self.h0q.pop()
            self.expand(n0)
            self.closed0.append(n0.state)


# State stores information for a given problem (Constructor used in respective planner)
class state:
  nq = None   #number of queens placed
  qs = None   #position of queens placed (by column) (redundant with path...)
  n =  None
  def __init__(self,nq,qs,n):
    self.qs = []
    for i in range(nq):
      self.qs.append(qs[i])
    self.qs += [-1]*(n-nq)
    self.nq = nq
    self.n = n
  def isEq(self,s) : return self.qs == s.qs

# Planner needs start, is_goal, cost, and succ functions implemented
class nqueens:
  n = 0
  def __init__(self,n):
    self.n = n
  
  def print(self,qs):
      print(qs)
      print(" ")
      for r in range(self.n):
          line = " "
          # Print horizontal dominos
          for c in range(self.n):
              if qs[c] == r: line += bcolors.OKRED + "\u265B" + bcolors.ENDC
              else: line += "\u2610"
              line += " "
          print(line)
      print(" ")

  def start(self):
    return state(0,[-1]*self.n,self.n)
  
  def is_goal(self,state):
    return state.nq == self.n
    
  def cost(self,s1,s2): return 1 # cost 1 to place queen
    
  def succ(self,s):
    qs = s.qs
    nq = s.nq
    succ = []
    for i in range(self.n):
      add = True
      for j in range(nq):
        if qs[j] == i or qs[j] == i+(nq-j) or qs[j] == i-(nq-j):
          add = False
          break
      qsP = qs[:]
      qsP[nq] = i
      if add: succ.append((state(nq+1,qsP,self.n),i))
    return succ
 
 
# Heuristics take state as input and return int
def h0(state):
  qs = state.qs
  nq = state.nq
  n = state.n
  return n - nq

# For test 1 showing case where SMHA worse than IMHA
def h1(state):
  qs = state.qs
  nq = state.nq
  n = state.n
  h = 0
  sol = [4, 1, 9, 6]
  for i in range(nq):
    if i > 3: break
    if qs[i] == sol[i]: h+=.2
  return n - nq - h
  
def h2(state):
  qs = state.qs
  nq = state.nq
  n = state.n
  h = 0
  sol = [1, 3, 0, 7]
#  if nq < 2: return n - nq + 1
  for i in range(2,nq):
    if i > 3: break
    if qs[i] == sol[i]: h+=.2 # stronger decrease
  return n - nq - h
  
# Presentation Heuristics:
# Need to be weighted appropriately so heuristics are similar
# with each other and with the anchor (whatever that might be)

# Distance from previously placed queen
def h_local_dist(state):
  qs = state.qs
  nq = state.nq
  n = state.n
  if nq == 1: return n - nq # only 1 queen
  pos_prev = qs[nq-2]
  pos_curr = qs[nq-1]
  return abs(pos_prev-pos_curr)+1
  
def h_mean_dist_local(state):
  qs = state.qs
  nq = state.nq
  n = state.n
  if nq == 1: return n - nq # only 1 queen
  dist = 0
  for i in range(nq-1):
    dist += abs(qs[i+1]-qs[i])+1
  return dist/(nq-1)
  
def h_open_squares(state):
#  return 0

  # Open squares can only occur after nq (all prev columns blocked)
  # Dumb implementation... loops through all squares after nq
  qs = state.qs
  nq = state.nq
  n = state.n
  squares = [[0]*n for i in range(n)]
  for i in range(nq):
    # block row, up diag, down diag
    for c in range(nq,n):
      squares[qs[i]][c] = 1 # row
      if qs[i]+(c-i) < n-1: squares[qs[i]+(c-i)][c] = 1 # upward diag
      if qs[i]-(c-i) > -1 : squares[qs[i]-(c-i)][c] = 1 # downward diag

  cnt = 0
  for c in range(nq,n):
    for r in range(n):
      if squares[r][c] == 0: cnt += 1
  return cnt
  
def h_mean_dist_all(state):
  qs = state.qs
  nq = state.nq
  n = state.n
  if nq == 1: return n - nq # only 1 queen
  dist = 0
  cnt = 0
  for i in range(nq-1):
    for j in range(i+1,nq):
      cnt += 1
      dist += abs(qs[j]-qs[i])+abs(j - i) # vert + hoiz distance
  return dist/cnt

    
def run(name, args):
    n = None
    t = 0 # Test 
    optlist, args = getopt.getopt(args, "n:t:")
    for (opt, val) in optlist:
        if opt == '-n':
            n = int(val)
        elif opt == '-t':
            t = int(val)
    if t == 0:
      w1 = 1
      w2 = 1
      
      nq = nqueens(n)
      solve = imha(nq,h0,[h1,h2],w1,w2)
      sol = solve.search(nq.start())
      nq.print(sol)
      print(solve.cnt)
      
      nq = nqueens(n)
      solve = smha(nq,h0,[h1,h2],w1,w2)
      sol = solve.search(nq.start())
      nq.print(sol)
      print(solve.cnt)
    elif t == 1:
      # heuristic test (n=8)
      s1 = state(1, [0] + [-1]*7, 8)
      s2 = state(2, [0,5] + [-1]*6, 8)
      s3 = state(3, [0,5,3] + [-1]*6, 8)
      print("s1 [0] h_local_dist {} h_mean_dist_local {} h_open_square {} h_mean_dist_all {} ".\
        format(h_local_dist(s1), h_mean_dist_local(s1), h_open_squares(s1), h_mean_dist_all(s1)))
      print("s2 [0,5] h_local_dist {} h_mean_dist_local {} h_open_square {} h_mean_dist_all {} ".\
        format(h_local_dist(s2), h_mean_dist_local(s2), h_open_squares(s2), h_mean_dist_all(s2)))
      print("s3 [0,5,3] h_local_dist {} h_mean_dist_local {} h_open_square {} h_mean_dist_all {} ".\
        format(h_local_dist(s3), h_mean_dist_local(s3), h_open_squares(s3), h_mean_dist_all(s3)))
      
      
    # Additional Tests here... T-2 Shows that SMHA cannot even find a solution
    elif t == 2:
      w1 = 1
      w2 = 1
      
      nq = nqueens(n)
      solve = imha(nq,h0,[h_local_dist,h_mean_dist_local],w1,w2)
      sol = solve.search(nq.start())
      nq.print(sol)
      print(solve.cnt)
      
      nq = nqueens(n)
      solve = smha(nq,h0,[h_local_dist,h_mean_dist_local],w1,w2)
      sol = solve.search(nq.start())
      nq.print(sol)
      print(solve.cnt)
    elif t == 3:
      w1 = 1
      w2 = 1
      
      nq = nqueens(n)
      solve = imha(nq,h0,[h_local_dist,h_open_squares],w1,w2)
      sol = solve.search(nq.start())
      nq.print(sol)
      print(solve.cnt)
      
      nq = nqueens(n)
      solve = smha(nq,h0,[h_local_dist,h_open_squares],w1,w2)
      sol = solve.search(nq.start())
      nq.print(sol)
      print(solve.cnt)


if __name__ == "__main__":
    run(sys.argv[0], sys.argv[1:])

# Stats - number of rounds printed after solution (can add counter and timers)

# Test 1
# Given example: IMHA 2xs better
# SMHA wants to splice both paths (see heuristics) which leads to less efficient solve time...
