// *** Hardwarespecific functions ***
void UTFT::_hw_special_init()
{
}

void UTFT::LCD_Writ_Bus(char VH,char VL, byte mode)
{   
	switch (mode)
	{
	case 1:
		if (display_serial_mode==SERIAL_4PIN)
		{
		if (VH==1)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		}
		else
		{
		if (VH==1)
			sbi(P_RS, B_RS);
		else
			cbi(P_RS, B_RS);
		}

		if (VL & 0x80)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		if (VL & 0x40)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		if (VL & 0x20)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		if (VL & 0x10)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		if (VL & 0x08)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		if (VL & 0x04)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		if (VL & 0x02)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		if (VL & 0x01)
			sbi(P_SDA, B_SDA);
		else
			cbi(P_SDA, B_SDA);
		pulse_low(P_SCL, B_SCL);
		break;
	case 8:
		LATECLR = 0xFF;
		LATESET = VH;
		*P_WR &= ~B_WR;
		*P_WR |= B_WR;
		LATECLR = 0xFF;
		LATESET = VL;
		*P_WR &= ~B_WR;
		*P_WR |= B_WR;
		break;
	case 16:
#ifdef AQUALED_SHIELD
		LATACLR = 0xFF;
		LATASET = VL & 0xFF;
#else
		LATDCLR = 0xFF;
		LATDSET = VL & 0xFF;
#endif
		LATECLR = 0xFF;
		LATESET = VH & 0xFF;
		*P_WR &= ~B_WR;
		*P_WR |= B_WR;
		break;
	case LATCHED_16:
		asm("nop");		// Mode is unsupported
		break;
	}
}

void UTFT::LCD_Write_Bus_8(char VL)
{
	LATECLR = 0xFF;
	LATESET = VL;
	*P_WR &= ~B_WR;
	*P_WR |= B_WR;
}

void UTFT::_set_direction_registers(byte mode)
{
	if (mode!=LATCHED_16)
	{
		TRISE=0;
		if (mode==16)
#ifdef AQUALED_SHIELD
			TRISA=0;
#else
			TRISD=0;
#endif
	}
	else
	{
		asm("nop");		// Mode is unsupported
	}
}

void UTFT::_fast_fill_16(int ch, int cl, long pix)
{
	long blocks;

#ifdef AQUALED_SHIELD
	LATACLR = 0xFF;
	LATASET = cl & 0xFF;
#else
	LATDCLR = 0xFF;
	LATDSET = cl & 0xFF;
#endif
	LATECLR = 0xFF;
	LATESET = ch & 0xFF;

	blocks = pix/16;
	for (int i=0; i<blocks; i++)
	{
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
		*P_WR &= ~B_WR;	*P_WR |= B_WR;
	}
	if ((pix % 16) != 0)
		for (int i=0; i<(pix % 16)+1; i++)
		{
			*P_WR &= ~B_WR;	*P_WR |= B_WR;
		}
}

void UTFT::_fast_fill_8(int ch, long pix)
{
	long blocks;

	LATECLR = 0xFF;
	LATESET = ch;

	blocks = pix/16;
	for (int i=0; i<blocks; i++)
	{
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
	}
	if ((pix % 16) != 0)
		for (int i=0; i<(pix % 16)+1; i++)
		{
			*P_WR &= ~B_WR; *P_WR |= B_WR; *P_WR &= ~B_WR; *P_WR |= B_WR;
		}
}
