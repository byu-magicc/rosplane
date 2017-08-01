#!/usr/bin/env python

################################################################################
#
# Copyright (c) 2017 BYU MAGICC Lab.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

import rospy
#from std_msgs.msg import String
from rosplane_msgs.msg import State
from rosgraph_msgs.msg import Clock

#stuff we need for plotting
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation

class state_subscriber():

	def __init__(self):
		#----------------------States-------------------------------
		self.pn = 0.
		self.pe = 0.
		self.pd = 0.
		self.Va = 0.
		self.alpha = 0.
		self.beta = 0.
		self.phi = 0.
		self.theta = 0.
		self.psi = 0.
		self.chi = 0.
		self.p = 0.
		self.q = 0.
		self.r = 0.
		self.Vg = 0.
		self.wn = 0.
		self.we = 0.
		self.time = 0.

		self.pnE = 0.
		self.peE = 0.
		self.pdE = 0.
		self.VaE = 0.
		self.phiE = 0.
		self.thetaE = 0.
		self.psiE = 0.

		#------------------------------------------------------------
		rospy.Subscriber("truth", State, self.callbackTruth)
		rospy.Subscriber("state", State, self.callbackEst)
		rospy.Subscriber("/clock", Clock, self.callback_time)
		self.rate = 100 # 100 hz

	def callbackTruth(self, State):
		self.pn = State.position[0]
		self.pe = State.position[1]
		self.pd = State.position[2]
		self.Va = State.Va
		self.alpha = State.alpha
		self.beta = State.beta
		self.phi = State.phi
		self.theta = State.theta
		self.psi = State.psi
		self.chi = State.chi
		self.p = State.p
		self.q = State.q
		self.r = State.r
		self.Vg = State.Vg
		self.wn = State.wn
		self.we = State.we

	def callbackEst(self, State):
		self.pnE = State.position[0]
		self.peE = State.position[1]
		self.pdE = State.position[2]
		self.VaE = State.Va
		self.alphaE = State.alpha
		self.betaE = State.beta
		self.phiE = State.phi
		self.thetaE = State.theta
		self.psiE = State.psi
		self.chiE = State.chi
		self.pE = State.p
		self.qE = State.q
		self.rE = State.r
		self.VgE = State.Vg
		self.wnE = State.wn
		self.weE = State.we

	def callback_time(self, Clock):
		self.time = Clock.clock.to_sec()

	def print_states(self):
		print "pn: ", self.pn
		print "pe: ", self.pe
		print "pd: ", self.pd
		print "Va: ", self.Va
		print "alpha: ", self.alpha
		print "beta: ", self.beta
		print "phi: ", self.phi
		print "theta: ", self.theta
		print "psi: ", self.psi
		print "chi: ", self.chi
		print "p: ", self.p
		print "q:", self.q
		print "r:", self.r
		print "Vg:", self.Vg
		print "wn:", self.wn
		print "we:", self.we
		if self.time == None:
			#do nothing
			eric = 7
		else:
			print "time: ", self.time.to_sec(), '\n'

rospy.init_node('FW_est_plot', anonymous=True)
states = state_subscriber()
#------------------------------------------------------------


# the following variables keep track of our axis limits so we can scale them when needed
pn_max	= 7.0
pn_min	=-7.0
pe_max	= 7.0
pe_min	=-7.0
pd_max	= 7.0
pd_min	=-7.0
Va_max	= 30.0
Va_min	= 0.0
alpha_max = 7.0
alpha_min =-7.0
beta_max  = 7.0
beta_min  =-7.0
phi_max   = 0.5
phi_min   =-0.5
theta_max = 0.5
theta_min =-0.5
psi_max   = 7.0
psi_min   =-7.0
chi_max   = 7.0
chi_min   =-7.0
p_max	 = 7.0
p_min	 =-7.0
q_max	 = 7.0
q_min	 =-7.0
r_max	 = 7.0
r_min	 =-7.0
Vg_max	= 7.0
Vg_min	=-7.0
wn_max	= 7.0
wn_min	=-7.0
we_max	= 7.0
we_min	=-7.0

# set up figure and animation for plotting of states
fig_plots1 = plt.figure()
#fig_plots2 = plt.figure()

ax_pn	= fig_plots1.add_subplot(811, xlim=(0, 4), ylim=(pn_min, pn_max))
ax_pe	= fig_plots1.add_subplot(812, sharex=ax_pn, ylim=(pe_min, pe_max))
ax_pd	= fig_plots1.add_subplot(813, sharex=ax_pn, ylim=(pd_min, pd_max))
ax_Va	= fig_plots1.add_subplot(814, sharex=ax_pn, ylim=(Va_min, Va_max))
#ax_alpha = fig_plots1.add_subplot(815, sharex=ax_pn, ylim=(-7, 7))
ax_phi = fig_plots1.add_subplot(815, sharex=ax_pn, ylim=(phi_min, phi_max))
#ax_beta  = fig_plots1.add_subplot(816, sharex=ax_pn, ylim=(-7, 7))
ax_theta  = fig_plots1.add_subplot(816, sharex=ax_pn, ylim=(theta_min, theta_max))
#ax_phi   = fig_plots1.add_subplot(817, sharex=ax_pn, ylim=(-7, 7))
ax_psi   = fig_plots1.add_subplot(817, sharex=ax_pn, ylim=(psi_min, psi_max))
#ax_theta = fig_plots1.add_subplot(818, sharex=ax_pn, ylim=(-7, 7))
#ax_psi   = fig_plots2.add_subplot(811, sharex=ax_pn, ylim=(-7, 7))
#ax_chi   = fig_plots2.add_subplot(812, sharex=ax_pn, ylim=(-7, 7))
#ax_p	 = fig_plots2.add_subplot(813, sharex=ax_pn, ylim=(-7, 7))
#ax_q	 = fig_plots2.add_subplot(814, sharex=ax_pn, ylim=(-7, 7))
#ax_r	 = fig_plots2.add_subplot(815, sharex=ax_pn, ylim=(-7, 7))
#ax_Vg	= fig_plots2.add_subplot(816, sharex=ax_pn, ylim=(-7, 7))
#ax_wn	= fig_plots2.add_subplot(817, sharex=ax_pn, ylim=(-7, 7))
#ax_we	= fig_plots2.add_subplot(818, sharex=ax_pn, ylim=(-7, 7))

ax_pn.grid()
ax_pe.grid()
ax_pd.grid()
ax_Va.grid()
#ax_alpha.grid()
#ax_beta.grid()
ax_phi.grid()
ax_theta.grid()
ax_psi.grid()
#ax_chi.grid()
#ax_p.grid()
#ax_q.grid()
#ax_r.grid()
#ax_Vg.grid()
#ax_wn.grid()
#ax_we.grid()

line_pn,line_pnE	= ax_pn.plot([], [], [], [], 'r')
line_pe,line_peE	= ax_pe.plot([], [], [], [], 'r')
line_pd,line_pdE	= ax_pd.plot([], [], [], [], 'r')
line_Va,line_VaE	= ax_Va.plot([], [], [], [], 'r')
#line_alpha, = ax_alpha.plot([], [])
#line_beta,  = ax_beta.plot([], [])
line_phi,line_phiE   = ax_phi.plot([], [], [], [], 'r')
line_theta,line_thetaE = ax_theta.plot([], [], [], [], 'r')
line_psi,line_psiE   = ax_psi.plot([], [], [], [], 'r')
#line_chi,   = ax_chi.plot([], [])
#line_p,	 = ax_p.plot([], [])
#line_q,	 = ax_q.plot([], [])
#line_r,	 = ax_r.plot([], [])
#line_Vg,	= ax_Vg.plot([], [])
#line_wn,	= ax_wn.plot([], [])
#line_we,	= ax_we.plot([], [])

ax_pn.set_ylabel('pn')
ax_pe.set_ylabel('pe')
ax_pd.set_ylabel('pd')
ax_Va.set_ylabel('Va')
#ax_alpha.set_ylabel(u'\u0391')
#ax_beta.set_ylabel(u'\u0392')
ax_phi.set_ylabel(u'\u03A6')
ax_theta.set_ylabel(u'\u0398')
ax_psi.set_ylabel(u'\u03C8')
#ax_chi.set_ylabel(u'\u03A7')
#ax_p.set_ylabel('p')
#ax_q.set_ylabel('q')
#ax_r.set_ylabel('r')
#ax_Vg.set_ylabel('Vg')
#ax_wn.set_ylabel('wn')
#ax_we.set_ylabel('we')

pn_data	 = np.array([])
pe_data	 = np.array([])
pd_data	 = np.array([])
Va_data	 = np.array([])
#alpha_data  = np.array([])
#beta_data   = np.array([])
phi_data	= np.array([])
theta_data  = np.array([])
psi_data	= np.array([])
#chi_data	= np.array([])
#p_data	  = np.array([])
#q_data	  = np.array([])
#r_data	  = np.array([])
#Vg_data	 = np.array([])
#wn_data	 = np.array([])
#we_data	 = np.array([])

pnE_data	 = np.array([])
peE_data	 = np.array([])
pdE_data	 = np.array([])
VaE_data	 = np.array([])
phiE_data	= np.array([])
thetaE_data  = np.array([])
psiE_data	= np.array([])

time_data   = np.array([])


axis_xlim = 0.0

def init_plot1():
	"""initialize animation"""
	line_pn.set_data([], [])
	line_pe.set_data([], [])
	line_pd.set_data([], [])
	line_Va.set_data([], [])
	#	line_alpha.set_data([], [])
	#	line_beta.set_data([], [])
	line_phi.set_data([], [])
	line_theta.set_data([], [])
	line_psi.set_data([], [])
	#	line_chi.set_data([], [])
	#	line_p.set_data([], [])
	#	line_q.set_data([], [])
	#	line_r.set_data([], [])
	#	line_Vg.set_data([], [])
	#	line_wn.set_data([], [])
	#	line_we.set_data([], [])

	line_pnE.set_data([], [])
	line_peE.set_data([], [])
	line_pdE.set_data([], [])
	line_VaE.set_data([], [])
	line_phiE.set_data([], [])
	line_thetaE.set_data([], [])
	line_psiE.set_data([], [])

	#	return line_pn, line_pe, line_pd, line_Va, line_alpha, line_beta, line_phi, line_theta
	return line_pn, line_pe, line_pd, line_Va, line_phi, line_theta, line_psi, line_pnE, line_peE, line_pdE, line_VaE, line_phiE, line_thetaE, line_psiE
def animate_plot1(i):
	"""perform animation step"""

	#	global states, pn_data, pe_data, pd_data, Va_data, alpha_data, beta_data, phi_data, theta_data, time_data
	global states, pn_data, pe_data, pd_data, Va_data, phi_data, theta_data, psi_data, time_data
	global pnE_data, peE_data, pdE_data, VaE_data, phiE_data, thetaE_data, psiE_data
	#	global ax_pn, ax_pe, ax_pd, ax_Va, ax_alpha, ax_beta, ax_phi, ax_theta, fig_plots1, fig_plots2
	global ax_pn, ax_pe, ax_pd, ax_Va, ax_phi, ax_theta, ax_psi, fig_plots1
	#	global pn_max, pn_min, pe_max, pe_min, pd_max, pd_min, Va_max, Va_min, alpha_max, alpha_min, beta_max, beta_min, phi_max, phi_min, theta_max, theta_min, axis_xlim
	global pn_max, pn_min, pe_max, pe_min, pd_max, pd_min, Va_max, Va_min, phi_max, phi_min, theta_max, theta_min, psi_max, psi_min, axis_xlim
	# the append function doesn't append to the array given by reference, so we have to pass it by value and simultaneously assign it to the original
	pn_data	 = np.append(pn_data, states.pn)
	pe_data	 = np.append(pe_data, states.pe)
	pd_data	 = np.append(pd_data, states.pd)
	Va_data	 = np.append(Va_data, states.Va)
	phi_data	= np.append(phi_data, states.phi)
	theta_data  = np.append(theta_data, states.theta)
	psi_data = np.append(psi_data, states.psi)

	pnE_data	 = np.append(pnE_data, states.pnE)
	peE_data	 = np.append(peE_data, states.peE)
	pdE_data	 = np.append(pdE_data, states.pdE)
	VaE_data	 = np.append(VaE_data, states.VaE)
	phiE_data	= np.append(phiE_data, states.phiE)
	thetaE_data  = np.append(thetaE_data, states.thetaE)
	psiE_data = np.append(psiE_data, states.psiE)
	time_data   = np.append(time_data, states.time)

	# update the time axis when necessary... they are all linked to the same pointer so you only need to update theta1
	need_to_plot = False
	if(states.time > axis_xlim):
		axis_xlim += 2.0 # this number moves the x axis of all plots by a given amount
		ax_pn.set_xlim(0, axis_xlim)
		need_to_plot = True

	# update the y-axis of each plot by a certain amount if the max or min is going off the plot
	# pn check
	if(pn_min > pn_data.min()):
		pn_min = pn_data.min() - 1.0
		ax_pn.set_ylim(pn_min, pn_max)
		need_to_plot = True

	if(pn_max < pn_data.max()):
		pn_max = pn_data.max() + 1.0
		ax_pn.set_ylim(pn_min, pn_max)
		need_to_plot = True

	# pe check
	if(pe_min > pe_data.min()):
		pe_min = pe_data.min() - 1.0
		ax_pe.set_ylim(pe_min, pe_max)
		need_to_plot = True

	if(pe_max < pe_data.max()):
		pe_max = pe_data.max() + 1.0
		ax_pe.set_ylim(pe_min, pe_max)
		need_to_plot = True

	# pd check
	if(pd_min > pd_data.min()):
		pd_min = pd_data.min() - 1.0
		ax_pd.set_ylim(pd_min, pd_max)
		need_to_plot = True

	if(pd_max < pd_data.max()):
		pd_max = pd_data.max() + 1.0
		ax_pd.set_ylim(pd_min, pd_max)
		need_to_plot = True

	# Va check
	if(Va_min > Va_data.min()):
		Va_min = Va_data.min() - 1.0
		ax_Va.set_ylim(Va_min, Va_max)
		need_to_plot = True

	if(Va_max < Va_data.max()):
		Va_max = Va_data.max() + 1.0
		ax_Va.set_ylim(Va_min, Va_max)
		need_to_plot = True

#	# alpha check
#	if(alpha_min > alpha_data.min()):
#		alpha_min = alpha_data.min() - 1.0
#		ax_alpha.set_ylim(alpha_min, alpha_max)
#		need_to_plot = True
#
#	if(alpha_max < alpha_data.max()):
#		alpha_max = alpha_data.max() + 1.0
#		ax_alpha.set_ylim(alpha_min, alpha_max)
#		need_to_plot = True
#
#	# beta check
#	if(beta_min > beta_data.min()):
#		beta_min = beta_data.min() - 1.0
#		ax_beta.set_ylim(beta_min, beta_max)
#		need_to_plot = True
#
#	if(beta_max < beta_data.max()):
#		beta_max = beta_data.max() + 1.0
#		ax_beta.set_ylim(beta_min, beta_max)
#		need_to_plot = True

    # phi check
	if(phi_min > phi_data.min()):
		phi_min = phi_data.min() - 1.0
		ax_phi.set_ylim(phi_min, phi_max)
		need_to_plot = True

	if(phi_max < phi_data.max()):
		phi_max = phi_data.max() + 1.0
		ax_phi.set_ylim(phi_min, phi_max)
		need_to_plot = True

	# theta check
	if(theta_min > theta_data.min()):
		theta_min = theta_data.min() - 1.0
		ax_theta.set_ylim(theta_min, theta_max)
		need_to_plot = True

	if(theta_max < theta_data.max()):
		theta_max = theta_data.max() + 1.0
		ax_theta.set_ylim(theta_min, theta_max)
		need_to_plot = True

	# psi check
	if(psi_min > psi_data.min()):
		psi_min = psi_data.min() - 1.0
		ax_psi.set_ylim(psi_min, psi_max)
		need_to_plot = True

	if(psi_max < psi_data.max()):
		psi_max = psi_data.max() + 1.0
		ax_psi.set_ylim(psi_min, psi_max)
		need_to_plot = True

	# update the plot if any of the axis limits have changed
	if need_to_plot:
		fig_plots1.show()

	line_pn.set_data(time_data,	 pn_data   )
	line_pe.set_data(time_data,	 pe_data   )
	line_pd.set_data(time_data,	 pd_data   )
	line_Va.set_data(time_data,	 Va_data   )
	line_phi.set_data(time_data,	phi_data  )
	line_theta.set_data(time_data,  theta_data)
	line_psi.set_data(time_data,  psi_data)
	line_pnE.set_data(time_data,	 pnE_data   )
	line_peE.set_data(time_data,	 peE_data   )
	line_pdE.set_data(time_data,	 pdE_data   )
	line_VaE.set_data(time_data,	 VaE_data   )
	line_phiE.set_data(time_data,	phiE_data  )
	line_thetaE.set_data(time_data,  thetaE_data)
	line_psiE.set_data(time_data,  psiE_data)
	return line_pn, line_pe, line_pd, line_Va, line_phi, line_theta, line_psi, line_pnE, line_peE, line_pdE, line_VaE, line_phiE, line_thetaE, line_psiE
'''
def init_plot2():
	"""initialize animation"""
	line_psi.set_data([], [])
	line_chi.set_data([], [])
	line_p.set_data([], [])
	line_q.set_data([], [])
	line_r.set_data([], [])
	line_Vg.set_data([], [])
	line_wn.set_data([], [])
	line_we.set_data([], [])
	return line_psi, line_chi, line_p, line_q, line_r, line_Vg, line_wn, line_we
'''

'''
def animate_plot2(i):
	"""perform animation step"""
	global states, psi_data, chi_data, p_data, q_data, r_data, Vg_data, wn_data, we_data, time_data
	global ax_psi, ax_chi, ax_p, ax_q, ax_r, ax_Vg, ax_wn, ax_we, fig_plots2
	global psi_max, psi_min, chi_max, chi_min, p_max, p_min, q_max, q_min, r_max, r_min, Vg_max, Vg_min, wn_max, wn_min, we_max, we_min, axis_xlim
	# the append function doesn't append to the array given by reference, so we have to pass it by value and simultaneously assign it to the original
	psi_data	= np.append(psi_data, states.psi)
	chi_data	= np.append(chi_data, states.chi)
	p_data	  = np.append(p_data, states.p)
	q_data	  = np.append(q_data, states.q)
	r_data	  = np.append(r_data, states.r)
	Vg_data	 = np.append(Vg_data, states.Vg)
	wn_data	 = np.append(wn_data, states.wn)
	we_data	 = np.append(we_data, states.we)
	# update the time axis when necessary... they are all linked to the same pointer so you only need to update theta1
	need_to_plot = False
	if(states.time > axis_xlim):
		axis_xlim += 2.0 # this number moves the x axis of all plots by a given amount
		ax_pn.set_xlim(0, axis_xlim)
		need_to_plot = True
	# update the y-axis of each plot by a certain amount if the max or min is going off the plot
	# psi check
	if(psi_min > psi_data.min()):
		psi_min = psi_data.min() - 1.0
		ax_psi.set_ylim(psi_min, psi_max)
		need_to_plot = True
	if(psi_max < psi_data.max()):
		psi_max = psi_min.max() + 1.0
		ax_psi.set_ylim(psi_min, psi_max)
		need_to_plot = True
	# chi check
	if(chi_min > chi_data.min()):
		chi_min = chi_data.min() - 1.0
		ax_chi.set_ylim(chi_min, chi_max)
		need_to_plot = True
	if(chi_max < chi_data.max()):
		chi_max = chi_data.max() + 1.0
		ax_chi.set_ylim(chi_min, chi_max)
		need_to_plot = True
	# p check
	if(p_min > p_data.min()):
		p_min = p_data.min() - 1.0
		ax_p.set_ylim(p_min, p_max)
		need_to_plot = True
	if(p_max < p_data.max()):
		p_max = p_data.max() + 1.0
		ax_p.set_ylim(p_min, p_max)
		need_to_plot = True
	# q check
	if(q_min > q_data.min()):
		q_min = q_data.min() - 1.0
		ax_q.set_ylim(q_min, q_max)
		need_to_plot = True
	if(q_max < q_data.max()):
		q_max = q_data.max() + 1.0
		ax_q.set_ylim(q_min, q_max)
		need_to_plot = True
	# r check
	if(r_min > r_data.min()):
		r_min = r_data.min() - 1.0
		ax_r.set_ylim(r_min, r_max)
		need_to_plot = True
	if(r_max < r_data.max()):
		r_max = r_min.max() + 1.0
		ax_r.set_ylim(r_min, r_max)
		need_to_plot = True
	# Vg check
	if(Vg_min > Vg_data.min()):
		Vg_min = Vg_data.min() - 1.0
		ax_Vg.set_ylim(Vg_min, Vg_max)
		need_to_plot = True
	if(Vg_max < Vg_data.max()):
		Vg_max = Vg_data.max() + 1.0
		ax_Vg.set_ylim(Vg_min, Vg_max)
		need_to_plot = True
	# wn check
	if(wn_min > wn_data.min()):
		wn_min = wn_data.min() - 1.0
		ax_wn.set_ylim(wn_min, wn_max)
		need_to_plot = True
	if(wn_max < wn_data.max()):
		wn_max = wn_data.max() + 1.0
		ax_wn.set_ylim(wn_min, wn_max)
		need_to_plot = True
	# we check
	if(we_min > we_data.min()):
		we_min = we_data.min() - 1.0
		ax_we.set_ylim(we_min, we_max)
		need_to_plot = True
	if(we_max < we_data.max()):
		we_max = we_data.max() + 1.0
		ax_we.set_ylim(we_min, we_max)
		need_to_plot = True
	# update the plot if any of the axis limits have changed
	if need_to_plot:
		fig_plots2.show()
	line_psi.set_data(time_data,	psi_data  )
	line_chi.set_data(time_data,	chi_data  )
	line_p.set_data(time_data,	  p_data	)
	line_q.set_data(time_data,	  q_data	)
	line_r.set_data(time_data,	  r_data	)
	line_Vg.set_data(time_data,	 Vg_data   )
	line_wn.set_data(time_data,	 wn_data   )
	line_we.set_data(time_data,	 we_data   )
	return line_psi, line_chi, line_p, line_q, line_r, line_Vg, line_wn, line_we
'''

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate_plot1(0)
#animate_plot2(0)
t1 = time()
interval = 1000 * (1./30.) - (t1 - t0)

ani1_plot = animation.FuncAnimation(fig_plots1, animate_plot1, frames=300,
								interval=interval, blit=True, init_func=init_plot1)
#ani2_plot = animation.FuncAnimation(fig_plots2, animate_plot2, frames=300,
#								interval=interval, blit=True, init_func=init_plot2)

plt.show()
'''
rate = rospy.Rate(states.rate)
try:
	while not rospy.is_shutdown():
		states.print_states()
		rate.sleep()
except rospy.ROSInterruptException:
	pass
'''
