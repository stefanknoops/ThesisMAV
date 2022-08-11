/**
 * This file is part of the dvs_of package - MAVLab TU Delft
 *
 *   MIT License
 *
 *   Copyright (c) 2020 MAVLab TU Delft
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *
 * */

#include <dvs_of/of_server.h>

namespace dvs_of
{

    dvs_of_msg::FlowPacketMsgArray PacketPub_;
    dvs_of_msg::VectorCount countpackage;

    /**
     * Function for implementing the binary search
     */
    uint32_t RateBuffer::find_closest_rate_idx(int64_t ts)
    {
        int32_t idx_upper = this->length_ - 1;
        int32_t idx_lower = 0;
        int32_t current_idx = idx_upper / 2;

        int32_t current_idx_wrapped = (this->buffer_idx_ - current_idx + this->length_) % this->length_;

        while (ts != this->buffer_[current_idx_wrapped].ts && idx_upper - idx_lower > 1)
        {
            if (ts < this->buffer_[current_idx_wrapped].ts)
            {
                idx_lower = current_idx;
                current_idx += (idx_upper - current_idx) / 2;
            }
            else
            {
                idx_upper = current_idx;
                current_idx += (idx_lower - current_idx) / 2;
            }
            current_idx_wrapped = (this->buffer_idx_ - current_idx + this->length_) % this->length_;
        }

        // Value absolutely closest to requested
        if (ts < this->buffer_[current_idx_wrapped].ts)
        {
            if (this->buffer_[current_idx_wrapped].ts - ts > fabsf((float)ts - (float)this->buffer_[(current_idx_wrapped - 1 + this->length_) % this->length_].ts))
            {
                current_idx_wrapped = (current_idx_wrapped - 1 + this->length_) % this->length_;
            }
        }
        else if (ts > this->buffer_[current_idx_wrapped].ts)
        {
            if (ts - this->buffer_[current_idx_wrapped].ts > fabsf((float)ts - (float)this->buffer_[(current_idx_wrapped + 1) % this->length_].ts))
            {
                current_idx_wrapped = (current_idx_wrapped + 1) % this->length_;
            }
        }
        // ROS_INFO("idx wrapped %i",current_idx_wrapped);
        return current_idx_wrapped;
    }

    rates_t RateBuffer::find_closest_rate(int64_t ts)
    {
        this->rates_mutex.lock();
        rates_t rate = this->buffer_[this->find_closest_rate_idx(ts)];
        this->rates_mutex.unlock();
        return rate;
    }

    /**
     * Get average rate from ts to now
     */
    rates_t RateBuffer::find_average_rate(int64_t ts)
    {
        // ROS_INFO("new average");
        rates_t rate = {0, 0, 0, 0};
        this->rates_mutex.lock();

        for (int i = 0; i < NewBuffer.size(); i++)
        {
            rate.p += NewBuffer[i].p;
            rate.q += NewBuffer[i].q;
            rate.r += NewBuffer[i].r;
        }

        rate.p /= NewBuffer.size();
        rate.q /= NewBuffer.size();
        rate.r /= NewBuffer.size();
        // std::cout << "rate p new " << rate.p << "\n";
        rate.ts = NewBuffer[NewBuffer.size() - 1].ts;

        this->rates_mutex.unlock();

        return rate;
    }

    int32_t RateBuffer::getLength()
    {
        return this->length_;
    }

    /**
     * RateBuffer: buffer containing the IMU data coming from the DVS
     */

    /**
     * Constructor (ON-LINE)
     */
    RateBuffer::RateBuffer(float time_window, float rate)
    {
        this->period_ = 1e6 / rate;
        this->length_ = (int32_t)(time_window * rate);
        this->buffer_ = (rates_t *)std::calloc(this->length_, sizeof(rates_t));
        // std::cout << "\t ... On-line processing set at: " << BoolToString(ON_OFF_PROC) << std::endl;
        ROS_INFO_STREAM(" ... On-line processing set at: " << BoolToString(ON_OFF_PROC));
    }

    /**
     * Constructor (OFF-LINE)
     */
    RateBuffer::RateBuffer(float time_window, float rate, uint32_t SizeIMU)
    {
        this->period_ = 1e6 / rate;
        this->length_ = SizeIMU;
        this->buffer_ = (rates_t *)std::calloc(this->length_, sizeof(rates_t));
        std::cout << "\t ... On-line processing set at: " << BoolToString(ON_OFF_PROC) << std::endl;
    }

    /**
     * Destructor
     */
    RateBuffer::~RateBuffer()
    {
        free(buffer_);
    }

    void RateBuffer::resetMedianFilters()
    {
        this->p_filt.~MedianFilter();
        this->q_filt.~MedianFilter();
        this->r_filt.~MedianFilter();
        this->p_filt = MedianFilter(this->num_ele);
        this->q_filt = MedianFilter(this->num_ele);
        this->r_filt = MedianFilter(this->num_ele);
    }

    void RateBuffer::log_rates(std::vector<IMU> *myIMU)
    {

        static bool initialized = false;
        static int init_counter = 0;
        static rates_t zero_rates = {0.f, 0.f, 0.f, 0};
        static int64_t last_ts = 0;

        // Pass gyro through median filter to remove outliers
        // std::cout << "\n\t ... Filtering IMU." << std::endl;
        // ROS_INFO_STREAM(" ... Filtering IMU.");

        std::vector<IMU>::iterator it = myIMU->begin();
        uint64_t t0 = (*it).t;
        uint64_t tf;
        for (; it != myIMU->end(); it++)
        {

            this->p_filt.update((*it).gyr_x); // pitch
            this->q_filt.update((*it).gyr_y); // yaw
            this->r_filt.update((*it).gyr_z); // roll

            if ((*it).t >= last_ts + this->period_ && initialized)
            {

                this->rates_mutex.lock();
                last_ts = (*it).t;
                /*
                //("buffer idx input %i",this->buffer_idx_);

                this->buffer_[this->buffer_idx_].p = this->p_filt.get_median() - zero_rates.p;
                this->buffer_[this->buffer_idx_].q = this->q_filt.get_median() - zero_rates.q;
                this->buffer_[this->buffer_idx_].r = this->r_filt.get_median() - zero_rates.r;
                this->buffer_[this->buffer_idx_].ts = last_ts;
                this->buffer_idx_ = (this->buffer_idx_ + 1) % this->length_;
                */
                rates_t newRates;
                newRates.p = this->p_filt.get_median() - zero_rates.p;
                newRates.q = this->q_filt.get_median() - zero_rates.q;
                newRates.r = this->r_filt.get_median() - zero_rates.r;
                newRates.ts = last_ts;
                NewBuffer.push_back(newRates);
                if (NewBuffer.size() > 5)
                {
                    NewBuffer.erase(NewBuffer.begin());
                }

                this->rates_mutex.unlock();
            }
            else if ((*it).t >= last_ts + this->period_)
            { // provide unfiltered value for now, big TODO to fix
                rates_t newRates;

                newRates.p = (*it).gyr_x;
                newRates.q = (*it).gyr_z;
                newRates.r = (*it).gyr_y;

                NewBuffer.push_back(newRates);
                if (NewBuffer.size() > 5)
                {
                    NewBuffer.erase(NewBuffer.begin());
                }

                this->rates_mutex.unlock();
            }
        }

        // Get average zero reading
        if (!initialized)
        {
            init_counter++;
            zero_rates.p += this->p_filt.get_median();
            zero_rates.q += this->q_filt.get_median();
            zero_rates.r += this->r_filt.get_median();
            if (init_counter >= 100) //>= 200)
            {

                zero_rates.p /= init_counter;
                zero_rates.q /= init_counter;
                zero_rates.r /= init_counter;
                initialized = true;
                ROS_INFO("Zero rates: %f %f %f", zero_rates.p, zero_rates.q, zero_rates.r);
            }
        }
    }

    /**
     * Function log_rates(std::vector<IMU> *myIMU) adapted for off-line processing
     */
    void RateBuffer::offline_log_rates(std::vector<IMU> *myIMU)
    {
        static rates_t zero_rates = {0.f, 0.f, 0.f, 0};

        // Pass gyro through median filter to remove outliers
        // std::cout << "\n\t ... Processing IMU data." << std::endl;
        std::vector<IMU>::iterator it = myIMU->begin();

        // Determine the zero_rates
        static int init_counter = 0;
        while (init_counter <= MAX_COUNTER && it != myIMU->end())
        {
            init_counter++;
            this->p_filt.update((*it).gyr_x);
            this->q_filt.update((*it).gyr_y);
            this->r_filt.update((*it).gyr_z);
            zero_rates.p += this->p_filt.get_median();
            zero_rates.q += this->q_filt.get_median();
            zero_rates.r += this->r_filt.get_median();
            it++;
        }
        zero_rates.p /= init_counter;
        zero_rates.q /= init_counter;
        zero_rates.r /= init_counter;

        // Apply the median filter on the IMU data-set
        this->resetMedianFilters();
        it = myIMU->begin();
        int64_t t0 = (*it).t;
        int64_t tf = 0;
        for (; it != myIMU->end(); it++)
        {
            this->p_filt.update((*it).gyr_x);
            this->q_filt.update((*it).gyr_y);
            this->r_filt.update((*it).gyr_z);
            tf = (*it).t;
            this->rates_mutex.lock();
            this->buffer_[this->buffer_idx_].p = RadOfDeg(this->p_filt.get_median() - zero_rates.p);
            this->buffer_[this->buffer_idx_].q = RadOfDeg(this->q_filt.get_median() - zero_rates.q);
            this->buffer_[this->buffer_idx_].r = RadOfDeg(this->r_filt.get_median() - zero_rates.r);
            this->buffer_[this->buffer_idx_].ts = tf;
            this->buffer_idx_ = (this->buffer_idx_ + 1) % this->length_;
            this->rates_mutex.unlock();
        }
    }

    /**
     * Initialize the flow state and parameters
     */
    void OpticFlow::initFlowState()
    {
        this->myFlowState.r = 2;
        this->myFlowState.minPixels = 8;
        this->myFlowState.refPeriod = 200000; // in microseconds
        this->myFlowState.lastEventT = 0;
        this->myFlowState.flowRate = 0.f;
        this->myFlowState.rateSetpoint = 4000.f;
        this->myFlowState.rateTimeConstant = 0.02f;
        this->myFlowState.dtMax = 1500000; // in microseconds
        this->myFlowState.maxNRMSE = 0.25f;
        this->myFlowState.dtStopFactor = 4.f;
        this->myFlowState.vMax = 20.f; // in rad/s
        this->myFlowState.nReject = 1;

        // Limit the events rate to make it processed real time
        // Not working if set to "true"
        this->myFlowState.limitEventRate = false;
        this->myFlowState.minDt = 1e06 / 1000;
    }

    /**
     * Determine the neighborhood mask
     */
    void OpticFlow::determinePixelNeighborhood()
    {
        this->myFlowState.kernelSize = (size_t)((this->myFlowState.r * 2 + 1) * (this->myFlowState.r * 2 + 1));
        this->myFlowState.dxKernel = new int16_t[this->myFlowState.kernelSize];
        this->myFlowState.dyKernel = new int16_t[this->myFlowState.kernelSize];
        int16_t x, y;
        size_t n = 0;
        for (x = -this->myFlowState.r; x <= this->myFlowState.r; x++)
        {
            for (y = -this->myFlowState.r; y <= this->myFlowState.r; y++)
            {
                this->myFlowState.dxKernel[n] = x;
                this->myFlowState.dyKernel[n] = y;
                n++;
            }
        }
    }

    /**
     * Store optic flow results
     */
    void OpticFlow::storeEventsFlow(double d_u, double d_v, double mag, rates_t rates, double rot_u, double rot_v, double chck)
    {

        if (!EventsFlow.is_open())
        {
            std::cout << "Unable to open output file: " << this->FileName << std::endl;
            exit(2);
        }
        else
        {
            EventsFlow << this->myFlowPacket.t << "," << this->myFlowPacket.x << "," << this->myFlowPacket.y << "," << this->myFlowPacket.p << ",";
            EventsFlow << this->myFlowPacket.u << "," << this->myFlowPacket.v << "," << rates.p << "," << rates.q << "," << rates.r << "," << d_u << "," << d_v << "," << rot_u << "," << rot_v << "," << rates.ts << std::endl;
        }
    }

    void OpticFlow::setLogFileName(std::string filename)
    {
        this->FileName = filename;
        // std::ofstream EventsFlow;
        EventsFlow.open(this->FileName);
    }

    /**
     * OpticFlow: contains the optic flow computed with the plane fitting algorithm
     */

    /**
     * Constructor (ON-LINE)
     */
    OpticFlow::OpticFlow(int w, int h)
    {
        this->width_ = w;
        this->height_ = h;
        this->start_time_ = clock();

        int min_val = w < h ? w : h;
        this->x_offset = (w - min_val) / 2;
        this->y_offset = (h - min_val) / 2;

        this->initFlowState();
        this->determinePixelNeighborhood();
        myRates = new RateBuffer(this->myFlowState.dtMax / 1e6f, 500);
    }

    /**
     * Constructor (OFF-LINE)
     */
    OpticFlow::OpticFlow(int w, int h, uint32_t SizeIMU)
    {
        this->width_ = w;
        this->height_ = h;
        this->start_time_ = clock();

        int min_val = w < h ? w : h;
        this->x_offset = (w - min_val) / 2;
        this->y_offset = (h - min_val) / 2;

        this->initFlowState();
        this->determinePixelNeighborhood();
        if (ON_OFF_PROC)
        {
            myRates = new RateBuffer(this->myFlowState.dtMax / 1e6f, 500);
        }
        else
        {
            myRates = new RateBuffer(this->myFlowState.dtMax / 1e6f, 500, SizeIMU);
        }
    }

    /**
     * Destructor
     */
    OpticFlow::~OpticFlow()
    {
        freeFlowStateMem(&this->myFlowState);
        EventsFlow.close();
    }

    void OpticFlow::checkVectorDirection(FlowPacket flow)
    {
        if (((flow.x < 120 && flow.u < 0) || (flow.x > 120 && flow.u >= 0)) && ((flow.y < 90 && flow.v < 0) || (flow.y > 90 && flow.v >= 0)))
        {
            countpackage.Forward++;
        }
        else if (((flow.x < 120 && flow.u > 0) || (flow.x > 120 && flow.u <= 0)) && ((flow.y < 90 && flow.v > 0) || (flow.y > 90 && flow.v <= 0)))
        {
            countpackage.Backward++;
        }
    }

    void OpticFlow::derotateFlow(dvs_of::FlowPacket FlowPacket)
    {
        derotate_flag = false;

        std::mutex derot_mutex;

        // float rotational_u = 0.f, rotational_v = 0.f;

        double x_nor, y_nor;

        double derotate_u = 0.f, derotate_v = 0.f;
        double derotate_mag = 0.0f;

        // Find rate in center of fitted plane

        rates = this->myRates->find_average_rate(FlowPacket.tP);

        // Derotate the flow
        // // Normalize the x,y coordinates
        // x_nor = 1.2*(((FlowPacket.x / 120.f) - 1.f));
        // y_nor = 1.2*(((FlowPacket.y / 90.f) - 1.f)); // scale as the image is not square

        x_nor = dvsGetUndistortedPixelX(FlowPacket.x, FlowPacket.y);
        y_nor = dvsGetUndistortedPixelY(FlowPacket.x, FlowPacket.y); // scale as the image is not square

        // std::cout << FlowPacket.x << ";" << x_nor << ";" << FlowPacket.y << ";" << y_nor << std::endl;

        rotational_u = -(-rates.q + rates.r * y_nor + rates.p * x_nor * y_nor - rates.q * x_nor * x_nor);
        rotational_v = -(rates.p - rates.r * x_nor - rates.q * x_nor * y_nor + rates.p * y_nor * y_nor);

        // std::cout << "rot_u = " << rotational_u << ", \t" << "rot_v = "<< rotational_v << std::endl;

        double mag_OF = 0.f, mag_OF_rot = 0.f, ang_OF = 0.f, ang_OF_rot = 0.f, OF_proj = 0.f, OF_der_pre = 0.f, OF_der = 0.f, u_der = 0.f, v_der = 0.f, pos_ang_OF_rot = 0.f, pos_ang_OF = 0.f;

        // Angle of optic flow
        ang_OF = atan2(FlowPacket.v, FlowPacket.u);

        // Angle of rotational flow
        ang_OF_rot = atan2(rotational_v, rotational_u);

        // Magnitude of optic flow
        mag_OF = hypot(FlowPacket.u, FlowPacket.v);

        // Magnitude of rotational flow
        mag_OF_rot = hypot(rotational_u, rotational_v);

        // Map to only positive angles
        if (ang_OF_rot < 0)
        {
            pos_ang_OF_rot = abs(2 * M_PI - abs(ang_OF_rot));
        }
        else
        {
            pos_ang_OF_rot = abs(ang_OF_rot);
        }

        if (ang_OF < 0)
        {
            pos_ang_OF = abs(2 * M_PI - abs(ang_OF));
        }
        else
        {
            pos_ang_OF = abs(ang_OF);
        }

        // Project rotational component onto optic flow
        OF_proj = mag_OF_rot * cos(abs(pos_ang_OF_rot - pos_ang_OF));
        OF_der = mag_OF - OF_proj;

        // Map only when rotational part does not exceed total flow
        //  if (OF_der_pre > 0)
        //  {
        //      OF_der = OF_der_pre;
        //  }
        //  else
        //  {
        //      OF_der = 0;
        //  }

        // Return to (u,v) notation
        this->u_der = OF_der * cos(pos_ang_OF);
        this->v_der = OF_der * sin(pos_ang_OF);

        // std::cout << "rot_u = " << rotational_u << ", \t" << "of u = "<< FlowPacket.u << std::endl;

        derotate_mag = rotational_u * rotational_u + rotational_v * rotational_v;

        if (derotate_mag > this->myFlowState.vMax * this->myFlowState.vMax)
        {
            this->derotate_flag = true;
        }
    }

    /**
     * Get the flow vectors
     */

    void OpticFlow::computeOpticFlowVectors(std::vector<Events> *myEventsFOV, std::vector<IMU> *myIMU)
    {

        std::vector<Events>::iterator it = myEventsFOV->begin();
        uint64_t t0 = (*it).t;
        uint64_t tf;
        Timer<std::chrono::microseconds> timer;
        float time_since_last = timer.toc();
        timer.tic();

        for (; it != myEventsFOV->end(); it++)
        {
            this->myFlowPacket.x = (*it).x;
            this->myFlowPacket.y = (*it).y;
            this->myFlowPacket.p = (*it).p;
            this->myFlowPacket.t = (*it).t;
            // software based refractory period check
            if (this->myFlowPacket.t < this->sae_on.buffer2D[this->myFlowPacket.x][this->myFlowPacket.y] + this->myFlowState.refPeriod ||
                this->myFlowPacket.t < this->sae_off.buffer2D[this->myFlowPacket.x][this->myFlowPacket.y] + this->myFlowState.refPeriod)
            {
                continue;
            }
            tf = (*it).t;
            if (this->myFlowPacket.p == 1)
            {
                computeOpticFlow(&this->myFlowPacket, &this->sae_on, &this->myFlowState);
            }
            else
            {
                computeOpticFlow(&this->myFlowPacket, &this->sae_off, &this->myFlowState);
            }
            if (this->myFlowPacket.valid)
            {
                float flow_mag = sqrtf(this->myFlowPacket.u * this->myFlowPacket.u + this->myFlowPacket.v * this->myFlowPacket.v);
            }
            else
            {
                this->total_flow_rejected_++;
                continue;
            }

            bool derotate = true;

            if (derotate)
            {
                derotateFlow(this->myFlowPacket);
            }
            else
            {
                u_der = this->myFlowPacket.u;
                v_der = this->myFlowPacket.v;
            }

            // d

            // Publish the Optic flow

            dvs_of_msg::FlowPacketMsg OFmsg_;
            OFmsg_.x = this->myFlowPacket.x;
            OFmsg_.y = this->myFlowPacket.y;
            OFmsg_.t = this->myFlowPacket.t;
            OFmsg_.p = this->myFlowPacket.p;
            OFmsg_.u = u_der;
            OFmsg_.v = v_der;
            OFmsg_.ru = this->myFlowPacket.u;
            OFmsg_.rv = this->myFlowPacket.v;

            PacketPub_.flowpacketmsgs.push_back(OFmsg_);
            PacketPub_.height = 180;
            PacketPub_.width = 240;

            // Output OF to .txt file
            this->storeEventsFlow(u_der, v_der, derotate_mag, rates, rotational_u, rotational_v, ang_OF);
            OpticFlow::checkVectorDirection(this->myFlowPacket);

            // Check if rotation is too large

            if (derotate_flag)
            {
                total_flow_rejected_++;
                continue;
            }
        };

        float comp_time = timer.toc();
        if (ON_OFF_PROC == true)
        {
            /// std::cout << "\t     -> Processing time: " << (comp_time - time_since_last)/SECONDS_TO_MICROSECONDS << " seconds" << std::endl;
        }
    }

    Server::Server(ros::NodeHandle &nh, ros::NodeHandle nh_private) : nh_(nh)
    {
        // Setup subscribers and publishers
        event_sub_ = nh_.subscribe("/dvs/events", 1, &Server::eventsCallback, this);
        imu_sub_ = nh_.subscribe("/dvs/imu", 1, &Server::imuCallback, this);
        foe_sub = nh_.subscribe("/FoEx", 1, &Server::foeCallback, this);

        OF_pub_ = nh_.advertise<dvs_of_msg::FlowPacketMsgArray>("/OpticFlow", 1);

        CountPublish = nh_.advertise<dvs_of_msg::VectorCount>("/CountVec", 1);

        if (nh_private.getParam("folder", folder))
        {
            ROS_INFO("Success in DVS_OF");
        };

        if (nh_private.getParam("calib", calib))
        {
            ROS_INFO("calibration file: %s", calib.c_str());
        };

        if (nh_private.getParam("orientation", orientation))
        {
            ROS_INFO("Camera orientation: %s", orientation.c_str());
        };

        int check = mkdir(folder.c_str(), 0777);

        initializeUmap(calib);

        // data record
        std::string myDate = currentDateTime();
        std::string ID1("DVS_recording");
        std::string ID2("IMU_recording");
        std::string ID3("OF_LOGFILE");
        std::string filename1 = folder + ID1 + ".txt";
        std::string filename2 = folder + ID2 + ".txt";
        std::string filename3 = folder + ID3 + ".txt";
        DVS_rec_file.open(filename1);
        IMU_rec_file.open(filename2);

        myOpticFlow = new OpticFlow(DIMX, DIMY);
        myOpticFlow->setLogFileName(filename3);
    }

    Server::~Server()
    {
        DVS_rec_file.close();
        IMU_rec_file.close();

        myOpticFlow->~OpticFlow();
    }

    void Server::foeCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        // Store FoE x coordinate in variable
        myOpticFlow->FoE_x = msg->data;
    }

    void Server::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        myIMU.clear();

        if (myIMU.empty())
        {
            IMU imu_;
            imu_.t = (uint64_t)(msg->header.stamp.toNSec() / 1000);
            if (imu_.t > 1e5)
            {
                imu_.acc_x = (double)(msg->linear_acceleration.x);
                imu_.acc_y = (double)(msg->linear_acceleration.y);
                imu_.acc_z = (double)(msg->linear_acceleration.z);

                if (orientation == "downward")
                {
                    imu_.gyr_x = -(float)(msg->angular_velocity.x);
                    imu_.gyr_y = (float)(msg->angular_velocity.y);
                    imu_.gyr_z = (float)(msg->angular_velocity.z);
                }
                else
                { // forward orientation
                    imu_.gyr_x = (float)(msg->angular_velocity.x);
                    imu_.gyr_y = (float)(msg->angular_velocity.z);
                    imu_.gyr_z = (float)(msg->angular_velocity.y);
                }
                myIMU.push_back(imu_);
            }
        }

        if (!myIMU.empty())
        {
            myOpticFlow->myRates->log_rates(&myIMU);
        }

        IMU_rec_file << (msg->header.stamp.toNSec() / 1000) << ",";
        IMU_rec_file << (double)msg->linear_acceleration.x << ",";
        IMU_rec_file << (double)msg->linear_acceleration.y << ",";
        IMU_rec_file << (double)msg->linear_acceleration.z << ",";
        IMU_rec_file << (float)msg->angular_velocity.x << ",";
        IMU_rec_file << (float)msg->angular_velocity.y << ",";
        IMU_rec_file << (float)msg->angular_velocity.z << std::endl;
    }

    void Server::eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
    {

        myEvents.clear();

        if (myEvents.empty())
        {
            for (int i = 0; i < msg->events.size(); ++i)
            {
                Events e;
                e.t = (int64_t)(msg->events[i].ts.toNSec() / 1000);
                if (e.t > 1e5)
                {
                    e.x = (uint8_t)(msg->events[i].x);
                    e.y = (uint8_t)(msg->events[i].y);
                    e.p = (uint8_t)(msg->events[i].polarity);
                    if (e.x > mX - X_FOV_RADIUS && e.x < mX + X_FOV_RADIUS)
                    {
                        if (e.y > mY - Y_FOV_RADIUS && e.y < mY + Y_FOV_RADIUS)
                        {
                            DVS_rec_file << e.t << ",";
                            DVS_rec_file << e.x << ",";
                            DVS_rec_file << e.y << ",";
                            DVS_rec_file << e.p << std::endl;
                            myEvents.push_back(e);
                        }
                    }
                }
            }

            if (!myEvents.empty())
            {

                myOpticFlow->computeOpticFlowVectors(&myEvents, &myIMU);

                OF_pub_.publish(PacketPub_);
                CountPublish.publish(countpackage);

                countpackage.Backward = 0;
                countpackage.Forward = 0;
                PacketPub_.flowpacketmsgs.clear();
            }
        }
    }

} // namespace