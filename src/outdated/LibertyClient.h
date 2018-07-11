/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** \file LibertyClient.h
 *
 *  \author Jonathan Scholz
 *  \date 7/17/2013
 */

#ifndef LIBERTYCLIENT_H_
#define LIBERTYCLIENT_H_

#include <amino.h>
#include <ach.h>
#include <somatic.h>
#include <somatic.pb-c.h>
#include <somatic/daemon.h>
#include <Eigen/Dense>

class LibertyClient {
public:
	LibertyClient();
	virtual ~LibertyClient();

	// initialization
	void initLiberty(somatic_d_t *daemon_cx, const char* channel_name = "liberty",
			size_t liberty_n_channels = 8, int *liberty_chan_ids = NULL);
	void setInitialPoses();

	// Update method
	bool updateRawPoses();
	void updateRelPoses();

	// Getters
	const std::vector<Eigen::Matrix4d>& getInitPoses() const {
		return initPoses;
	}

	const std::vector<Eigen::Matrix4d>& getRawPoses(bool update = true) {
		if (update)
			updateRawPoses();
		return rawPoses;
	}

	const std::vector<Eigen::Matrix4d>& getRelPoses(bool update = true) {
		if (update) {
			updateRawPoses();
			updateRelPoses();
		}

		return relPoses;
	}

protected:
	// Channel members
	somatic_d_t *daemon_cx;
	uint8_t *achbuf_liberty;
	static const size_t n_achbuf_liberty = 1024;
	ach_channel_t liberty_ach_chan;

	Eigen::VectorXi liberty_chan_ids;
	std::vector<Eigen::Matrix4d> initPoses;
	std::vector<Eigen::Matrix4d> rawPoses;
	std::vector<Eigen::Matrix4d> relPoses;
};

#endif /* LIBERTYCLIENT_H_ */