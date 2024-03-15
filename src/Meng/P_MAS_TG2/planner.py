# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
from ts import distance, reach_waypoint
from collections import defaultdict


from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal
from discrete_plan import improve_plan_given_history, mip
from discrete_plan import shortest_path_ts

class ltl_planner(object):
	def __init__(self, ts, hard_spec, soft_spec):
		buchi = mission_to_buchi(hard_spec, soft_spec)
		self.product = ProdAut(ts, buchi)
		self.time = 0
		self.pose = None
		self.trace = [] # log the regions been visited
		self.traj = [] # log the full trajectory
		self.opt_log = [] 
		self.com_log = []
		self.detour = None
		self.dindex = 0
		self.old_move = None
		self.path = None
		self.contract_time = 0 # time bound to help others

	def optimal(self, beta=1, style='static', segment='lasso'):
		self.beta = beta
		if style == 'static':
			# full graph construction
			self.product.graph['ts'].build_full()
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, beta=self.beta, segment=segment)
		elif style == 'on-the-fly':
			# on-the-fly construction
			self.product.build_initial()
			self.product.build_accept()
			self.run, plantime = dijkstra_plan_optimal(self.product, beta=self.beta, segment=segment)
		### show the results
		print 'the plan prefix:\n'
		print [n for n in self.run.pre_plan]
		print 'the plan suffix:\n'
		print [n for n in self.run.suf_plan]
		self.opt_log.append((self.time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
		self.last_time = self.time
		self.acc_change = 0
		self.index = 1
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
		return plantime

	def find_next_move(self):
		if not self.detour:
			if self.segment == 'line' and self.index < len(self.run.pre_plan)-1:
				self.trace.append(self.run.line[self.index])
				self.index += 1
				self.next_move = self.run.pre_plan[self.index]
			elif self.segment == 'line' and self.index == len(self.run.pre_plan)-1:
				self.trace.append(self.run.line[self.index])
				self.index = 0
				self.segment = 'loop'
				self.next_move = self.run.suf_plan[self.index]
			elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-1:
				self.trace.append(self.run.loop[self.index+1])
				self.index += 1
				self.next_move = self.run.suf_plan[self.index]
			elif self.segment == 'loop' and self.index == len(self.run.suf_plan)-1:
				self.trace.append(self.run.loop[self.index+1])
				self.index = 0
				self.next_move = self.run.suf_plan[self.index]
		else:
			if self.dindex < len(self.detour)-1:
				self.trace.append(self.detour[self.dindex])
				self.dindex += 1
				self.next_move = self.detour[self.dindex][0]
			elif self.dindex == len(self.detour)-1:
				self.trace.append(self.detour[self.dindex])
				#print self.detour[self.dindex]
				self.next_move = self.detour[self.dindex][1]
				self.dindex += 1
			else:
				self.detour = []
				if self.segment == 'line':
					self.next_move = self.run.pre_plan[self.index][:]
				elif self.segment == 'loop':
					self.next_move = self.run.suf_plan[self.index][:]


	def update(self,object_name):
		MotionFts = self.product.graph['ts'].graph['region']
		cur_region = MotionFts.closest_node(self.pose)
		sense_info = dict()
		sense_info['label'] = set([(cur_region,set([object_name,]),set()),]) 
		changes = MotionFts.update_after_region_change(sense_info,None)
		if changes:
			return True

	def replan(self, segment='lasso'):
		if (self.segment =='line') and (self.index > len(self.run.pre_plan)-3):
			print 'No need to change plan!'
		else:
			new_run = improve_plan_given_history(self.product, self.trace, pose=self.pose, segment=segment)
			if new_run:
				print new_run.pre_plan
			if (new_run) and (new_run.pre_plan !=self.run.pre_plan[(self.index):]):
				self.run = new_run
				if self.run.pre_plan:
					self.index = 0
					self.segment = 'line'
					self.next_move = self.run.pre_plan[self.index]
					print '************************'
					print 'Plan adapted!'
					print '************************'
				else:
					self.index = 0
					self.segment = 'loop'
					self.next_move = self.run.suf_plan[self.index]
					print 'Start plan suffix!'
			else:
				print 'Plan unchanged!'
	
	#========================
	# cooperative actions
	def cooperative_action_in_horizon(self, dep, horizon):
		k = 0
		j = self.index
		request = dict()
		if (self.contract_time<=0) and (self.segment == 'line'):
			while (k<horizon) and (j<len(self.run.pre_plan_cost)-1):
					k += self.run.pre_plan_cost[j]
					if self.run.pre_plan[j] in dep:
						for a_d in dep[self.run.pre_plan[j]]:
							request[(self.run.line[j][0],a_d)] = k
						return request
					j += 1
		return request
	
	def evaluate_request(self, request, alpha=1):
		reply = dict()
		path = dict()
		for t_ts_node, time in request.iteritems():
			ts = self.product.graph['ts']
			if (self.contract_time)>0 or (t_ts_node not in ts):
				reply[t_ts_node] = (False, 0)
			else:
				f_ts_node = self.run.line[self.index] 
				path[t_ts_node], cost = shortest_path_ts(ts, f_ts_node, t_ts_node)
				if path[t_ts_node] and (cost>0):
					reply[t_ts_node] = (True, cost + alpha*abs(cost-time))
				else:
					reply[t_ts_node] = (False, 0)
		self.path = path.copy()
		return reply

	def confirmation(self, request, Reply):
		# show the layout
		Confirm, time = mip(request, Reply)
		if time:
			self.contract_time = time 
		return Confirm, time

	def adapt_plan(self, confirm):
		if all(item[0]==False for item in confirm.itervalues()):
			pass
		else:
			for t_ts_node, b in confirm.iteritems():
				if b[0] and (t_ts_node in self.path):
					p = self.path[t_ts_node]
					self.detour = p
					self.next_move = p[0][0]
					self.dindex = 0
					self.contract_time = b[1]
					return True

	def delay_cooperation(self, delay, speed):
		f_ts_node = self.run.line[self.index]
		new_ts_node = (f_ts_node[0], 'None')
		p = [new_ts_node,]*int(round(delay*speed))
		self.detour = p
		self.next_move = p[0][0]
		self.dindex = 0
		#self.contract_time = delay
		self.contract_time = -delay














