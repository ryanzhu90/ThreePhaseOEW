#include <brtenv.h>    /* basic real-time environment */
#include <const.h>    /* general header for motor control */
#include <DS1104_CS_PWM7.c>
#include <Usrdsp.h>
#include <Firmware\Slv1104_fw240.slc>
#include <UserFunc.h>
#include <variables.h>
#include <varinit.c>
void PWM_sync_interrupt(void)//60us
{
	RTLIB_TIC_START();          /* start time measurement */
	host_service(1, 0);         //ControlDesk service 10us

	if (rst==1)  //reset loop
	{
		varinit();
		// Meausreing ADC Offset
		ds1104_adc_read_mux(scantable,4,adc_inv2);//Channel 1 2 3 4 for DC voltage and phase current of inverter 2
		ADC1_Ofst=GAIN_ADC1*adc_inv2[0];//ADC channel 1 offset
		ADC2_Ofst=GAIN_ADC2*adc_inv2[1];//ADC channel 2 offset
		ADC3_Ofst=GAIN_ADC3*adc_inv2[2];//ADC channel 3 offset
		ADC4_Ofst=GAIN_ADC4*adc_inv2[3];//ADC channel 4 offset
		ADC5_Ofst=GAIN_ADC5*ds1104_adc_read_ch_immediately(5);
		ADC6_Ofst=GAIN_ADC6*ds1104_adc_read_ch_immediately(6);
		ADC7_Ofst=GAIN_ADC7*ds1104_adc_read_ch_immediately(7);
		ADC8_Ofst=GAIN_ADC8*ds1104_adc_read_ch_immediately(8);
		//moving average filter disabled during HF INJECTION
		ADC1_sum=ADC1_sum+ADC1_Ofst-ADC1_buff[0];
		ADC2_sum=ADC2_sum+ADC2_Ofst-ADC2_buff[0];
		ADC3_sum=ADC3_sum+ADC4_Ofst-ADC3_buff[0];
		ADC4_sum=ADC4_sum+ADC4_Ofst-ADC4_buff[0];
		ADC5_sum=ADC5_sum+ADC5_Ofst-ADC5_buff[0];
		ADC6_sum=ADC6_sum+ADC6_Ofst-ADC6_buff[0];
		ADC7_sum=ADC7_sum+ADC7_Ofst-ADC7_buff[0];
		ADC8_sum=ADC8_sum+ADC8_Ofst-ADC8_buff[0];
		if(j==4)//read ADC every 4*TS 
		{
			j=0;
			noise_window=20;
			for (i=0;i<noise_window-1;i++)
			{
				ADC1_buff[i]=ADC1_buff[i+1];
				ADC2_buff[i]=ADC2_buff[i+1];
				ADC3_buff[i]=ADC3_buff[i+1];
				ADC4_buff[i]=ADC4_buff[i+1];
				ADC5_buff[i]=ADC5_buff[i+1];
				ADC6_buff[i]=ADC6_buff[i+1];
				ADC7_buff[i]=ADC7_buff[i+1];
				ADC8_buff[i]=ADC8_buff[i+1];
			}
			ADC1_buff[i]=ADC1_Ofst;
			ADC2_buff[i]=ADC2_Ofst;
			ADC3_buff[i]=ADC3_Ofst;
			ADC4_buff[i]=ADC4_Ofst;
			ADC5_buff[i]=ADC5_Ofst;
			ADC6_buff[i]=ADC6_Ofst;
			ADC7_buff[i]=ADC7_Ofst;
			ADC8_buff[i]=ADC8_Ofst;

			ADC1_Ofst=ADC1_sum/noise_window;
			ADC2_Ofst=ADC2_sum/noise_window;
			ADC3_Ofst=ADC3_sum/noise_window;
			ADC4_Ofst=ADC4_sum/noise_window;
			ADC5_Ofst=ADC5_sum/noise_window;
			ADC6_Ofst=ADC6_sum/noise_window;
			ADC7_Ofst=ADC7_sum/noise_window;
			ADC8_Ofst=ADC8_sum/noise_window;
		}
		else j=j+1;
	}
	else
	{
		//----------------- Measuring Position from Incremental Encoder -------------------
		inc_curr= ds1104_inc_position_read_immediately(1,DS1104_INC_LINE_SUBDIV_4)*PI2/INC_LINES;//current position in radius
		theta_rm =-(inc_curr +INIT_POS_OFFSET); //adding initial position to mechanical position;
		if(theta_rm<0.0) theta_rm=theta_rm+PI2;
		else if(theta_rm>=PI2) theta_rm=theta_rm-PI2;
		theta_re = Pp * theta_rm;//mech -> elec position
		if(theta_re<0.0) theta_re=theta_re+PI2;
		else if(theta_re>=PI2) theta_re=theta_re-PI2;

	//------------ Reading ADC -------------
	ds1104_adc_read_mux(scantable, 4, adc_inv2);//Channel 1 2 3 4 for DC voltage and phase current of inverter 2
	Vdc=GAIN_ADC1*adc_inv2[0]-ADC1_Ofst;//340V max DC
	if(Vdc<0.1)Vdc=0.1;
	ia=GAIN_ADC5*ds1104_adc_read_ch_immediately(5)-ADC5_Ofst;//phase current of inverter 1
	ib=GAIN_ADC6*ds1104_adc_read_ch_immediately(6)-ADC6_Ofst;//phase current of inverter 1
	ic=GAIN_ADC7*ds1104_adc_read_ch_immediately(7)-ADC7_Ofst;//phase current of inverter 1
	
	if(ia>0.02)sign_ia=1.0;
	else if(ia<-0.02)sign_ia=-1.0; 
	//else sign_ia=0.0;
	if(ib>0.02)sign_ib=1.0;
	else if(ib<-0.02)sign_ib=-1.0;
	//else sign_ib=0.0;
	if(ic>0.02)sign_ic=1.0;
	else if(ic<-0.02)sign_ic=-1.0;
	
	
	if(Vdc>Vdc_max) Vdc_stat=Vdc_stat+1;//Over-voltage Protection

	if(fabs(idc)>Idc_max) Idc_stat=Idc_stat+1;//Over-current Protection

	if(fabs(ia)>Iabc_max) ia_stat=ia_stat+1;//Over-current Protection

	if(fabs(ib)>Iabc_max) ib_stat=ib_stat+1;//Over-current Protection

	if(fabs(ic)>Iabc_max) ic_stat=ic_stat+1;//Over-current Protection

	if(ia_stat>4||ib_stat>4||ic_stat>4||Idc_stat>4||Vdc_stat>0) rst=1;

	ial=2.0/3.0*(ia-0.5*ib-0.5*ic); 
	ibe=SQRT1_3*(ib-ic);
	io=((1.0/3.0)*(ia+ib+ic));   //I zore current

	//average the io current
/* 	for (i=0;i<noise_window-1;i++)
	{
		io_buf[i]=io_buf[i+1];
	}
	io_buf[noise_window]=io;
	io_sum=io_sum+io_buf[noise_window]-io_buf[0];
	io_avg=io_sum/noise_window; */
	
	//IPMSM
	id=ial*cos(theta_re)+ibe*sin(theta_re);
	iq=-ial*sin(theta_re)+ibe*cos(theta_re);
	V_DC=2.0/3.0*Vdc; //SVPWM
	
	//----------------Calculate Electromagnetic torque--------------
	Te=1.5*Pp*(FLUXM*iq+(Ld-Lq)*id*iq);

	//----------------MTPA trajectory---------------
	id_max=FLUXM/4.0/(Lq-Ld)-sqrt(FLUXM*FLUXM/16.0/(Lq-Ld)/(Lq-Ld)+Iam*Iam/2.0); //Iam ^2 = iq^2+id^2
	iq_max=sqrt(Iam*Iam-id_max*id_max);
	
	////--------------For the ALL PASS FILTER-------------- 
	fr=wm_inc*Pp/(2.0*PI); //electrical
	fol_fr_apf.APF1 = ((6.0*fr*TS)-2.0)/((6.0*fr*TS)+2.0);
	apf(io,APFoutput,fol_fr_apf);
	zero_axis_estimator=atan2(APFoutput,io); //angle of the zero-axis estimator

		//---------Calculate the speed of the motor -------------------------
		// taking speed measurement every 8 cycles to reduce inaccuracy  
		index_inc=ds1104_inc_index_read(1,DS1104_INC_IDXMODE_ON);//A or B
		count++;
	if (count == spd_counter)//every four cycles
	{
		if(fabs(inc_curr-inc_old)>=PI||index_inc==1) {}	
		else inc_delta=inc_curr-inc_old;
		inc_old=inc_curr;
		wm_inc=-inc_delta/(TS*spd_counter);//rad/s
		count = 0;
	}
	
	//-------Fist Order Lag to remove the noise from the encoder ----------------
t_f=fol_w_inc_t;
fol_w_inc.K1 = (2.0*t_f-TS)/(2.0*t_f+TS);
fol_w_inc.K2 = TS/(2.0*t_f+TS); 
fol(wm_inc, wm_inc_lpf, fol_w_inc);	

	//-----------------------PI Speed Controller--------------------------
	Pulse_spd.frq=spd_ref_freq;
	Pulse_spd.amp=spd_ref_amp;
	Pulse_spd.ofs=spd_ref_ofs;
	Pulse_spd.dut=spd_ref_dut;
	PulseGen_MACRO(Pulse_spd);
	wm_ref=Pulse_spd.out*PI_30;//convert rpm to rad/s
	//iq_max=2.0;
		
		Ctrl_spd.ref=wm_ref; 		// Input: reference set-point
		Ctrl_spd.fbk=wm_inc;//wm_inc_lpf;			// Input: feedback
		//Ctrl_spd.fbk=wm_adp_lpf;//Enable at Sensorless
		Ctrl_spd.Kp=Kp_w;			// Parameter: proportional loop gain
		Ctrl_spd.Ki=Ki_w;
		Ctrl_spd.Kb=Kb_w;          // Parameter: anti-windup back-calculation gain
		Ctrl_spd.Umax=100.0*iq_max;		// Parameter: upper saturation limit
		Ctrl_spd.Umin=-100.0*iq_max;		// Parameter: lower saturation limit
		PI_BC_MACRO(Ctrl_spd);
	
	//iq_ref=Ctrl_spd.out/100.0; //why there is a gain of 100?
	Pulse_iq.frq=iq_ref_freq;
	Pulse_iq.amp=iq_ref_amp;
	Pulse_iq.ofs=iq_ref_ofs;
	Pulse_iq.dut=iq_ref_dut;
	PulseGen_MACRO(Pulse_iq);
	iq_ref=Pulse_iq.out;
	
	
	//----------------------PI q-Current Controllers for IPMSM --------------------------
	if (iq_ref >= iq_max) iq_ref = iq_max;
	else if (iq_ref <=-iq_max) iq_ref =-iq_max;
	
	Ctrl_iq.ref=iq_ref; 		// Input: reference set-point
	Ctrl_iq.fbk=iq;	

	Ctrl_iq.Kp=Kp_iq;		// Input: feedback
	Ctrl_iq.Ki=Ki_iq;
	Ctrl_iq.Kb=Kb_iq;          // Parameter: anti-windup back-calculation gain
	Ctrl_iq.Umax=V_DC;		// Parameter: upper saturation limit
	Ctrl_iq.Umin=-V_DC;		// Parameter: lower saturation limit
	PI_BC_MACRO(Ctrl_iq);	
	uc_iq=Ctrl_iq.out;
	
	//----------------------PI d-Current Controllers for IPMSM --------------------------
	id_ref=0.0;

	//MTPA trajectory
	//id_ref=FLUXM/2.0/(Lq-Ld)-sqrt(iq_ref*iq_ref+FLUXM*FLUXM/4.0/(Lq-Ld)/(Lq-Ld)); //how the reference is calculated
	if (id_ref <= id_max) id_ref = id_max;//id_max is negative

	Ctrl_id.ref=id_ref; 		// Input: reference set-point
	Ctrl_id.fbk=id;			// Input: feedback for id

	Ctrl_id.Kp=Kp_id;			// Parameter: proportional loop gain
	Ctrl_id.Ki=Ki_id;		    // Parameter: integral gain
	Ctrl_id.Kb=Kb_id;          // Parameter: anti-windup back-calculation gain
	Ctrl_id.Umax=V_DC;		// Parameter: upper saturation limit
	Ctrl_id.Umin=-V_DC;		// Parameter: lower saturation limit
	PI_BC_MACRO(Ctrl_id);	
	uc_id=Ctrl_id.out;

	//----------------------PI 0-current Controllers for IPMSM -------------------------
	Pulse_io.frq=io_ref_freq;
	Pulse_io.amp=io_ref_amp;
	Pulse_io.ofs=io_ref_ofs;
	Pulse_io.dut=io_ref_dut;
	PulseGen_MACRO(Pulse_io);
	io_ref=Pulse_io.out;
	
	Ctrl_io.ref=io_ref; 		// Input: reference set-point
	//Ctrl_io.fbk=iom_inc_lpf;			// Input: feedback for io
	Ctrl_io.fbk=io;//io_avg;
	Ctrl_io.Kp=Kp_io;			// Parameter: proportional loop gain
	Ctrl_io.Ki=Ki_io;		    // Parameter: integral gain
	Ctrl_io.Kb=Kb_io;          // Parameter: anti-windup back-calculation gain
	Ctrl_io.Umax=vdcFrac*V_DC;		// Parameter: upper saturation limit
	Ctrl_io.Umin=-vdcFrac*V_DC;		// Parameter: lower saturation limit
	PI_BC_MACRO(Ctrl_io)
	uc_io=Ctrl_io.out;

	//-------------Decoupling IPMSM--------------//
	Ud_ref=uc_id-Pp*wm_inc*Lq*iq;  //first term is the vd voltage from pi controller. second term is add as pi controller cannot control iq.
	Uq_ref=uc_iq+Pp*wm_inc*(FLUXM+Ld*id);
	Uo_ref=(uc_io+3.0*Pp*wm_inc*FLUX3M*sin (3*theta_re))*Uo_coef;
	
	//-------------Ual_ref and Ube_ref for IPMSM--------------//
	Ual_ref=Ud_ref*cos(theta_re)-Uq_ref*sin(theta_re);//for SVPWM
	Ube_ref=Ud_ref*sin(theta_re)+Uq_ref*cos(theta_re);//for SVPWM
	
	//------------- Output Voltage Limit for SVPWM-----------------
	Vomax=U_MAX_LINE*sqrt(2.0/3.0);//Vomax is max phase peak
	if(0.866*V_DC<Vomax)Vomax=0.866*V_DC;	
	Vo_mag=sqrt(Ual_ref*Ual_ref+Ube_ref*Ube_ref);//the magnitude of output voltage vector
	if(Vo_mag>=Vomax) //
	{
	Ual_ref=Vomax*Ual_ref/Vo_mag;
	Ube_ref=Vomax*Ube_ref/Vo_mag;
	Uo_ref=Vomax*Uo_ref/Vo_mag;
	Vo_mag=Vomax;
	} 
	m=SQRT3*Vo_mag/(1.5*V_DC);
	theta_Vo=atan2(Ube_ref,Ual_ref);//[-PI,PI)
	if(theta_Vo<0.0)theta_Vo=theta_Vo+PI2;
	
	if(theta_Vo>=0.0 && theta_Vo<PI_3)
	{
		sector=1;//V0(000),V1(100),V2(110),V7(111)
		d1=m*sin(PI_3-theta_Vo); //V1(1/3)tL        //V4(2/3)tH
		d2=m*sin(theta_Vo);          //V2(2/3)tH      //V5(1/3)tL
		deltaT=(Uo_ref/Vdc)-(d2/3.0)+(d1/3.0);
		
		//d0_1=0;
		//d0_2=0;
		d0_1=0.5*deltaT+0.5*(1.0-d1-d2);
		d0_2=-0.5*deltaT+0.5*(1.0-d1-d2);

		duty[0]=1.0-0.5*d0_1;//V1(100),V2(110),V7(111)   
		duty[1]=0.5*d0_1+d2;//V2(110),V7(111)
		duty[2]=0.5*d0_1;//V7(111)
		duty[3]=0.5*d0_2;//V7(111)
		duty[4]=0.5*d0_2+d1;//V4(011),V7(111)
		duty[5]=1.0-0.5*d0_2;//V5(001),V4(011),V7(111)
	}
	else if(theta_Vo>=PI_3 && theta_Vo<PI2_3)
	{
		sector=2;//V0(000),V3(010),V2(110),V7(111)
		theta_Vo=theta_Vo-PI_3;
		d1=m*sin(PI_3-theta_Vo); //V1(1/3)tL        //V4(2/3)tH
		d2=m*sin(theta_Vo);          //V2(2/3)tH      //V5(1/3)tL
		deltaT=(Uo_ref/Vdc)-(d2/3.0)+(d1/3.0);
		//d0_1=0;
		//d0_2=0;
		d0_1=0.5*deltaT+0.5*(1.0-d1-d2);
		d0_2=-0.5*deltaT+0.5*(1.0-d1-d2);
		duty[0]=0.5*d0_1+d1;//d1x+d1y;//V1(100)    //V1(100) V2(110) 
		duty[1]=1.0-0.5*d0_1;//d1y;//V2(110)
		duty[2]=0.5*d0_1;//V7(111)
		duty[3]=d2+0.5*d0_2;//0;//
		duty[4]=0.5*d0_2;;//d2x;//V4(011)
		duty[5]=1.0-0.5*d0_2;//V5(001)  //V5(001),V4(011)
	}
	else if(theta_Vo>=PI2_3 && theta_Vo<PI)
	{
		sector=3;//V0(000),V3(010),V4(011),V7(111)
		theta_Vo=theta_Vo-PI2_3;
		d1=m*sin(PI_3-theta_Vo); //V1(1/3)tL        //V4(2/3)tH
		d2=m*sin(theta_Vo);          //V2(2/3)tH      //V5(1/3)tL
		deltaT=(Uo_ref/Vdc)-(d2/3.0)+(d1/3.0);
		//d0_1=0;
		//d0_2=0;

		d0_1=0.5*deltaT+0.5*(1.0-d1-d2);
		d0_2=-0.5*deltaT+0.5*(1.0-d1-d2);
		duty[0]=0.5*d0_1;//V1(100)    //V1(100) V2(110) 
		duty[1]=1.0-0.5*d0_1;//V2(110)
		duty[2]=d2+0.5*d0_1;//V7(111)
		duty[3]=1.0-0.5*d0_2;//
		duty[4]=0.5*d0_2;//V4(011)
		duty[5]=d1+0.5*d0_2;//V5(001)  //V5(001),V4(011)
	}
	else if(theta_Vo>=PI && theta_Vo<PI4_3)
	{
		sector=4;//V0(000),V5(001),V4(011),V7(111)
		theta_Vo=theta_Vo-PI;
		d1=m*sin(PI_3-theta_Vo); //V1(1/3)tL        //V4(2/3)tH
		d2=m*sin(theta_Vo);          //V2(2/3)tH      //V5(1/3)tL
		deltaT=(Uo_ref/Vdc)-(d2/3.0)+(d1/3.0);
		//d0_1=0;
		//d0_2=0;
		d0_1=0.5*deltaT+0.5*(1.0-d1-d2);
		d0_2=-0.5*deltaT+0.5*(1.0-d1-d2);
		duty[0]= d0_1*0.5;//V7(111)
		duty[1]=d1+0.5* d0_1;//V3(011),V7(111)
		duty[2]=1.0-0.5* d0_1;//V5(001),V4(011),V7(111)
		duty[3]=1.0-0.5* d0_2;//V1(100),V2(110),V7(111)
		duty[4]=d2+0.5* d0_2;//V2(110),V7(111)
		duty[5]=0.5* d0_2;//V7(111)
	}
	else if(theta_Vo>=PI4_3 && theta_Vo<PI5_3)
	{
		sector=5;//V0(000),V5(001),V6(101),V7(111)
		theta_Vo=theta_Vo-PI4_3;
		d1=m*sin(PI_3-theta_Vo); //V1(1/3)tL        //V4(2/3)tH
		d2=m*sin(theta_Vo);          //V2(2/3)tH      //V5(1/3)tL
		deltaT=(Uo_ref/Vdc)-(d2/3.0)+(d1/3.0);
		//d0_1=0;
		//d0_2=0;
		d0_1=0.5*deltaT+0.5*(1.0-d1-d2);
		d0_2=-0.5*deltaT+0.5*(1.0-d1-d2);
		duty[0]=d2+ d0_1*0.5;//V6(101),V7(111)
		duty[1]=0.5* d0_1;//V7(111)
		duty[2]=1.0-0.5* d0_1;//V5(001),V6(101),V7(111)
		duty[3]=d1+ d0_2*0.5;//V2(110),V7(111)
		duty[4]=1.0-0.5* d0_2;//V3(010),V2(110),V7(111)
		duty[5]=0.5* d0_2;//V7(111)
	}
	else
	{
		sector=6;//V0(000),V1(100),V6(101),V7(111)
		theta_Vo=theta_Vo-PI5_3;
		d1=m*sin(PI_3-theta_Vo); //V1(1/3)tL        //V4(2/3)tH
		d2=m*sin(theta_Vo);          //V2(2/3)tH      //V5(1/3)tL
		deltaT=(Uo_ref/Vdc)-(d2/3.0)+(d1/3.0);
		//d0_1=0;
		//d0_2=0;
		d0_1=0.5*deltaT+0.5*(1.0-d1-d2);
		d0_2=-0.5*deltaT+0.5*(1.0-d1-d2);
		duty[0]=1- d0_1*0.5;//V1(100),V6(101),V7(111)
		duty[1]=0.5* d0_1;//V7(111)
		duty[2]=d1+0.5*d0_1;//V6(101),V7(111)
		duty[3]= d0_2*0.5;//V7(111)
		duty[4]=1.0-0.5*d0_2;//V3(010),V4(011),V7(111)
		duty[5]=d2+0.5*d0_2;//V4(011),V7(111)
	}
	
	
	
	// activate the previously written DAC values synchronously 
	ds1104_dac_strobe();
	ds1104_bit_io_write(DAT_IO1+DAT_IO2*16);//enale 12 SW signals for two inverters
	ds1104_slave_dsp_pwm7_update(duty);//2.3us

}
	exec_time =RTLIB_TIC_READ();
}

main()
{
	init ();

	/* load the user firmware to the slave DSP TMS320F240 */
	ds1104_slave_dsp_appl_load((Int32 *) &fw240);

	/*initial the master-slave communication*/
	ds1104_slave_dsp_communication_init();
	varinit();
	/* sets IO0 to IO9 to output and IO10 to IO19 to input */
	ds1104_bit_io_init(DS1104_DIO0_OUT | DS1104_DIO1_OUT |
	DS1104_DIO2_OUT | DS1104_DIO3_OUT |
	DS1104_DIO4_OUT | DS1104_DIO5_OUT |
	DS1104_DIO6_OUT | DS1104_DIO7_OUT |
	DS1104_DIO8_IN | DS1104_DIO9_IN |
	DS1104_DIO10_IN | DS1104_DIO11_IN |
	DS1104_DIO12_IN | DS1104_DIO13_IN |
	DS1104_DIO14_IN | DS1104_DIO15_IN |
	DS1104_DIO16_IN | DS1104_DIO17_IN |
	DS1104_DIO18_IN | DS1104_DIO19_IN);

	/* writing on pins configured as input has no effect */
	ds1104_bit_io_write(0x00000000);//enale 12 SW signals for two inverters
	/* init D/A converter in latched mode */
	ds1104_dac_init(DS1104_DACMODE_LATCHED);
	 
	/*init incremental encoder channel 1*/
	  ds1104_inc_init(1, DS1104_INC_MODE_RS422);
	ds1104_inc_set_idxmode(1, DS1104_INC_IDXMODE_ON); //DS1104_INC_IDXMODE_ON);   
	//ds1104_inc_set_idxmode(1, DS1104_INC_IDXMODE_OFF); 
	ds1104_syncout_edge_setup(DS1104_SYNC_TRIGGER_FALLING);
	ds1104_inc_trigger_setup(1,DS1104_TRIGGER_ENABLE);
	ds1104_dac_trigger_setup(DS1104_TRIGGER_ENABLE);
	ds1104_adc_trigger_setup(2,DS1104_TRIGGER_ENABLE);
	ds1104_adc_trigger_setup(3,DS1104_TRIGGER_ENABLE);
	ds1104_adc_trigger_setup(4,DS1104_TRIGGER_ENABLE);
	ds1104_adc_trigger_setup(5,DS1104_TRIGGER_ENABLE);
	ds1104_syncin_trigger();

	ds1104_slave_dsp_pwm7_init(period, duty, sym_mode, update_mode, polarity, intpos);
	ds1104_slave_dsp_pwm7_start();//start PWM generation on all 7 channels
	ds1104_slave_dsp_pwm7_int_init(PWM_sync_interrupt);
	ds1104_enable_hardware_int(DS1104_INT_SLAVE_DSP_PWM);//slave DSP interrupt PWM generation
	RTLIB_INT_ENABLE();

	/* Background tasks */
	  while(1)
	{
		   RTLIB_BACKGROUND_SERVICE(); /* ControlDesk service */
	}

}
