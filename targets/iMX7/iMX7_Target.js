

function Connect (target)
{
	TargetInterface.message ("## Connect J-Link "  + target);
	TargetInterface.selectDevice (0, 0, 0, 0);	// irPre, irPost, drPre, drPost
	TargetInterface.setDeviceTypeProperty (target);
	TargetInterface.setDebugInterfaceProperty ("set_adiv5_APB_ap_num", 2);
	TargetInterface.setDebugInterfaceProperty ("set_adiv5_AHB_ap_num", 3);	
	TargetInterface.scanIR (4, 1);	// Scans the IR with Length of 4
	TargetInterface.message ("## Connect J-Link finished");
} 

function Reset ()
{
	TargetInterface.message ("## Reset Target");	
	TargetInterface.resetAndStop (1000);
	TargetInterface.pokeWord (0xE000ED94, 0x00000000);	// Disabling Cortex-M4 MPU
	TargetInterface.message ("## Reset Target end");
}


function GetPartName ()
{    
	var PART = "";
	return PART;
}

function MatchPartName (name)
{
	var partName = GetPartName();

	if (partName == "")
		return false;

	return partName.substring(0, 6) == name.substring(0, 6);
}

function EnableTrace(traceInterfaceType)
{
  // TODO: Enable trace
}

function Clock_Init ()
{
	// Enable all clocks
	TargetInterface.pokeWord (0x020c4068,0xffffffff);
	TargetInterface.pokeWord (0x020c406c,0xffffffff);
	TargetInterface.pokeWord (0x020c4070,0xffffffff);
	TargetInterface.pokeWord (0x020c4074,0xffffffff);
	TargetInterface.pokeWord (0x020c4078,0xffffffff);
	TargetInterface.pokeWord (0x020c407c,0xffffffff);
	TargetInterface.pokeWord (0x020c4080,0xffffffff);

	TargetInterface.message ("Clock Init Done");
}

function DDR_Init ()
{
	// Config IOMUX for ddr
	TargetInterface.pokeWord (0x020E04B4,0x000C0000);
	TargetInterface.pokeWord (0x020E04AC,0x00000000);
	TargetInterface.pokeWord (0x020E027C,0x00000030);
	TargetInterface.pokeWord (0x020E0250,0x00000030);
	TargetInterface.pokeWord (0x020E024C,0x00000030);
	TargetInterface.pokeWord (0x020E0490,0x00000030);
	TargetInterface.pokeWord (0x020E0288,0x00000030);
	TargetInterface.pokeWord (0x020E0270,0x00000000);
	TargetInterface.pokeWord (0x020E0260,0x00000030);
	TargetInterface.pokeWord (0x020E0264,0x00000030);
	TargetInterface.pokeWord (0x020E04A0,0x00000030);
	TargetInterface.pokeWord (0x020E0494,0x00020000);
	TargetInterface.pokeWord (0x020E0280,0x00000030);
	TargetInterface.pokeWord (0x020E0284,0x00000030);
	TargetInterface.pokeWord (0x020E04B0,0x00020000);
	TargetInterface.pokeWord (0x020E0498,0x00000030);
	TargetInterface.pokeWord (0x020E04A4,0x00000030);
	TargetInterface.pokeWord (0x020E0244,0x00000030);
	TargetInterface.pokeWord (0x020E0248,0x00000030);

	// Config DDR Controller Registers
	TargetInterface.pokeWord (0x021B001C,0x00008000);
	TargetInterface.pokeWord (0x021B0800,0xA1390003);
	TargetInterface.pokeWord (0x021B080C,0x00000000);
	TargetInterface.pokeWord (0x021B083C,0x41570155);
	TargetInterface.pokeWord (0x021B0848,0x4040474A);
	TargetInterface.pokeWord (0x021B0850,0x40405550);
	TargetInterface.pokeWord (0x021B081C,0x33333333);
	TargetInterface.pokeWord (0x021B0820,0x33333333);
	TargetInterface.pokeWord (0x021B082C,0xf3333333);
	TargetInterface.pokeWord (0x021B0830,0xf3333333);
	TargetInterface.pokeWord (0x021B08C0,0x00921012);
	TargetInterface.pokeWord (0x021B08b8,0x00000800);

	// Config MMDC init
	TargetInterface.pokeWord (0x021B0004,0x0002002D);
	TargetInterface.pokeWord (0x021B0008,0x1B333030);
	TargetInterface.pokeWord (0x021B000C,0x676B52F3);
	TargetInterface.pokeWord (0x021B0010,0xB66D0B63);
	TargetInterface.pokeWord (0x021B0014,0x01FF00DB);
	TargetInterface.pokeWord (0x021B0018,0x00201740);
	TargetInterface.pokeWord (0x021B001C,0x00008000);
	TargetInterface.pokeWord (0x021B002C,0x000026D2);
	TargetInterface.pokeWord (0x021B0030,0x006B1023);
	TargetInterface.pokeWord (0x021B0040,0x0000004F);
	TargetInterface.pokeWord (0x021B0000,0x84180000);
	TargetInterface.pokeWord (0x021B0890,0x23400A38);
	TargetInterface.pokeWord (0x021B001C,0x02008032);
	TargetInterface.pokeWord (0x021B001C,0x00008033);
	TargetInterface.pokeWord (0x021B001C,0x00048031);
	TargetInterface.pokeWord (0x021B001C,0x15208030);
	TargetInterface.pokeWord (0x021B001C,0x04008040);
	TargetInterface.pokeWord (0x021B0020,0x00000800);
	TargetInterface.pokeWord (0x021B0818,0x00000227);
	TargetInterface.pokeWord (0x021B0004,0x0002552D);
	TargetInterface.pokeWord (0x021B0404,0x00011006);
	TargetInterface.pokeWord (0x021B001C,0x00000000);

	TargetInterface.message ("DDR Init Done");
}