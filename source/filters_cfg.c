/*
 * filters_cfg.c
 *
 *  Created on: 11 sie 2023
 *      Author: Łukasz
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "filters_cfg.h"
#include "fsl_powerquad.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t fir_num_blocks = (BUFFER_SIZE / FIR_BLOCK_SIZE / 2);
static uint32_t iir_num_blocks = (BUFFER_SIZE / IIR_BLOCK_SIZE / 2);

#if PQ_USED
#if !(Q31_USED)
static float32_t src_state_buffer[BUFFER_SIZE +  2 * (FIR_COEFF_COUNT - 1)];
static float32_t dst_state_buffer[BUFFER_SIZE +  2 * (FIR_COEFF_COUNT - 1)];
#else
static q31_t src_state_buffer[BUFFER_SIZE +  2 * (FIR_COEFF_COUNT - 1)];
static q31_t dst_state_buffer[BUFFER_SIZE +  2 * (FIR_COEFF_COUNT - 1)];
#endif
#endif

const float32_t fir_coeff_f32[FIR_COEFF_COUNT] = {
	/* cutoff 6kHz, order 256 */
	0.0f, -0.000142095463926298f, -0.000203598484145306f, -0.000146364768649777f, 0.0f, 0.000152796255334117f, 0.000221823870954137f, 0.000161487217690435f, 0.0f, -0.000172536476232354f, -0.000253111426525135f, -0.000186044545777425f, 0.0f, 0.000202113832835479f, 0.000298599184600296f, 0.000220848866463926f, 0.0f, -0.000242356566655504f, -0.000359470896364611f, -0.000266746554889788f, 0.0f, 0.000294131512141478f, 0.000436968307791359f, 0.000324627590413839f, 0.0f, -0.000358354884777625f, -0.000532407533612327f, -0.000395437973970450f, 0.0f, 0.000436006538883024f, 0.000647200648234585f, 0.000480196069766373f, 0.0f, -0.000528148674789083f, -0.000782883983192874f, -0.000580014004716558f, 0.0f, 0.000635950311052743f, 0.000941155138348767f, 0.000696125658073407f, 0.0f, -0.000760719312910043f, -0.00112392144914783f, -0.000829923342363559f, 0.0f, 0.000903944450630234f, 0.00133336371102959f, 0.000983006098849127f, 0.0f, -0.00106735095564239f, -0.00157202051065915f, -0.00115724373801470f, 0.0f, 0.00125297451462210f, 0.00184290081687561f, 0.00135486255918409f, 0.0f, -0.00146326086157541f, -0.00214963597346358f, -0.00157856142905363f, 0.0f, 0.00170120154169663f, 0.00249668762992459f, 0.00183167116649932f, 0.0f, -0.00197052178649341f, -0.00288963667437853f, -0.00211837696848963f, 0.0f, 0.00227594508079392f, 0.00333559205593175f, 0.00244403468771380f, 0.0f, -0.00262357330092913f, -0.00384378142477216f, -0.00281563037311102f, 0.0f, 0.00302144569694921f, 0.00442642515110156f, 0.00324246475256718f, 0.0f, -0.00348038307159067f, -0.00510006593656665f, -0.00373720242153848f, 0.0f, 0.00401530265745672f, 0.00588765744666140f, 0.00431753458898457f, 0.0f, -0.00464734140520535f, -0.00682197091934482f, -0.00500891946123627f, 0.0f, 0.00540743416951147f, 0.00795140479312829f, 0.00584931437171450f, 0.0f, -0.00634266063239365f, -0.00935043979531675f, -0.00689782244903868f, 0.0f, 0.00752823196298245f, 0.0111397418830013f, 0.00825163780957785f, 0.0f, -0.00909198266861255f, -0.0135281916709367f, -0.0100823536463049f, 0.0f, 0.0112697959941345f, 0.0169109477103547f, 0.0127235246748694f, 0.0f, -0.0145497050974079f, -0.0221356107387184f, -0.0169198775073432f, 0.0f, 0.0201303896832348f, 0.0314071709056526f, 0.0247410730777657f, 0.0f, -0.0319516492901427f, -0.0528127241883479f, -0.0448815117930890f, 0.0f, 0.0749686340967730f, 0.159142724192092f, 0.225155374452901f, 0.250119395289229f, 0.225155374452901f, 0.159142724192092f, 0.0749686340967730f, 0.0f, -0.0448815117930890f, -0.0528127241883479f, -0.0319516492901427f, 0.0f, 0.0247410730777657f, 0.0314071709056526f, 0.0201303896832348f, 0.0f, -0.0169198775073432f, -0.0221356107387184f, -0.0145497050974079f, 0.0f, 0.0127235246748694f, 0.0169109477103547f, 0.0112697959941345f, 0.0f, -0.0100823536463049f, -0.0135281916709367f, -0.00909198266861255f, 0.0f, 0.00825163780957785f, 0.0111397418830013f, 0.00752823196298245f, 0.0f, -0.00689782244903868f, -0.00935043979531675f, -0.00634266063239365f, 0.0f, 0.00584931437171450f, 0.00795140479312829f, 0.00540743416951147f, 0.0f, -0.00500891946123627f, -0.00682197091934482f, -0.00464734140520535f, 0.0f, 0.00431753458898457f, 0.00588765744666140f, 0.00401530265745672f, 0.0f, -0.00373720242153848f, -0.00510006593656665f, -0.00348038307159067f, 0.0f, 0.00324246475256718f, 0.00442642515110156f, 0.00302144569694921f, 0.0f, -0.00281563037311102f, -0.00384378142477216f, -0.00262357330092913f, 0.0f, 0.00244403468771380f, 0.00333559205593175f, 0.00227594508079392f, 0.0f, -0.00211837696848963f, -0.00288963667437853f, -0.00197052178649341f, 0.0f, 0.00183167116649932f, 0.00249668762992459f, 0.00170120154169663f, 0.0f, -0.00157856142905363f, -0.00214963597346358f, -0.00146326086157541f, 0.0f, 0.00135486255918409f, 0.00184290081687561f, 0.00125297451462210f, 0.0f, -0.00115724373801470f, -0.00157202051065915f, -0.00106735095564239f, 0.0f, 0.000983006098849127f, 0.00133336371102959f, 0.000903944450630234f, 0.0f, -0.000829923342363559f, -0.00112392144914783f, -0.000760719312910043f, 0.0f, 0.000696125658073407f, 0.000941155138348767f, 0.000635950311052743f, 0.0f, -0.000580014004716558f, -0.000782883983192874f, -0.000528148674789083f, 0.0f, 0.000480196069766373f, 0.000647200648234585f, 0.000436006538883024f, 0.0f, -0.000395437973970450f, -0.000532407533612327f, -0.000358354884777625f, 0.0f, 0.000324627590413839f, 0.000436968307791359f, 0.000294131512141478f, 0.0f, -0.000266746554889788f, -0.000359470896364611f, -0.000242356566655504f, 0.0f, 0.000220848866463926f, 0.000298599184600296f, 0.000202113832835479f, 0.0f, -0.000186044545777425f, -0.000253111426525135f, -0.000172536476232354f, 0.0f, 0.000161487217690435f, 0.000221823870954137f, 0.000152796255334117f, 0.0f, -0.000146364768649777f, -0.000203598484145306f, -0.000142095463926298f, 0.0f
};
const float32_t iir_df1_coeff_f32[IIR_COEFF_COUNT] = {
	/* cutoff 10kHz, order 66
	 * b0	b1	b2	a1	a2 */
	0.035988055f,	0.07197611f,	0.035988055f,	0.416100782f,	-0.792953711f,
	0.24593792f,	0.491875841f,	0.24593792f,	0.358093443f,	-0.543003515f,
	0.292326008f,	0.584652016f,	0.292326008f,	0.316575117f,	-0.364103495f,
	0.323231352f,	0.646462704f,	0.323231352f,	0.286635666f,	-0.235096169f,
	0.341744243f,	0.683488487f,	0.341744243f,	0.265219174f,	-0.142813768f,
	0.349087413f,	0.698174825f,	0.349087413f,	0.250389528f,	-0.078913699f,
	0.342757509f,	0.685515018f,	0.342757509f,	0.240928402f,	-0.038146263f,
	0.255047221f,	0.510094441f,	0.255047221f,	0.23611087f,	-0.017387806f,
	0.196856394f,	0.393712788f,	0.196856394f,	0.235320385f,	-0.01398165f,
	0.202708174f,	0.405416347f,	0.202708174f,	0.23796906f,	-0.025394635f,
	0.212657948f,	0.425315897f,	0.212657948f,	0.24504296f,	-0.055875655f,
	0.227492558f,	0.454985116f,	0.227492558f,	0.257070705f,	-0.107702499f,
	0.24848634f,	0.49697268f,	0.24848634f,	0.275003464f,	-0.184973696f,
	0.277649477f,	0.555298955f,	0.277649477f,	0.300381713f,	-0.294327073f,
	0.318175645f,	0.63635129f,	0.318175645f,	0.335635454f,	-0.446233364f,
	0.116557064f,	0.233114129f,	0.116557064f,	0.384625349f,	-0.657327928f,
	0.108749869f,	0.217499739f,	0.108749869f,	0.453650093f,	-0.954751477f,
	0.198213273f,	0.396426545f,	0.198213273f,	0.399681548f,	-0.722204202f,
	0.262320034f,	0.524640069f,	0.262320034f,	0.34640137f,	-0.492623058f,
	0.308360525f,	0.61672105f,	0.308360525f,	0.308148545f,	-0.327793894f,
	0.340936401f,	0.681872801f,	0.340936401f,	0.280573359f,	-0.208974044f,
	0.3629516f,		0.725903201f,	0.3629516f,		0.260951915f,	-0.124426401f,
	0.376206256f,	0.752412511f,	0.376206256f,	0.247556479f,	-0.066706258f,
	0.367310471f,	0.734620942f,	0.367310471f,	0.239308211f,	-0.031164959f,
	0.202325709f,	0.404651417f,	0.202325709f,	0.23558349f,	-0.015115356f,
	0.200883361f,	0.401766722f,	0.200883361f,	0.236904876f,	-0.02080913f,
	0.209750263f,	0.419500527f,	0.209750263f,	0.242837043f,	-0.046370487f,
	0.223266419f,	0.446532838f,	0.223266419f,	0.253555762f,	-0.092556816f,
	0.242566376f,	0.485132752f,	0.242566376f,	0.269894712f,	-0.162960388f,
	0.269444337f,	0.538888674f,	0.269444337f,	0.293225388f,	-0.263490892f,
	0.306739667f,	0.613479334f,	0.306739667f,	0.325716703f,	-0.403494051f,
	0.2281868f,		0.4563736f,		0.2281868f,		0.370801013f,	-0.59775968f,
	0.39337002f,	0.786740039f,	0.39337002f,	0.434032986f,	-0.870222522f,
	/* overall system gain in the first section */
//		0.00000000000000000000447592720026990f,	0.0000000000000000000105999083667931f,	0.00000000000000000000359642187697879f,	0.497499972082111f,	-0.0162669755315006f,
//		1.0f,	7.47558438974580f,	14.0633957297544f,	 0.0658907870167683f,	-0.00135903856517598f,
//		1.0f,	7.09592816335954f,	13.3320865130273f,	 0.0518708471516507f,	-0.00179118819462542f,
//		1.0f,	6.46000699406580f,	12.1324293944563f,	 0.0262558442111451f,	-0.00279300692269661f,
//		1.0f,	5.70612628829083f,	10.7820743123391f,	 -0.0169620965409923f,	-0.00515502813753627f,
//		1.0f,	4.87520265507134f,	9.33867770996118f,	 -0.0856617737502446f,	-0.0123536777515204f,
//		1.0f,	4.01287832819333f,	7.80504829118311f,	 -0.154007093944427f,	-0.0307357081030066f,
//		1.0f,	3.21182292073734f,	6.31148538199818f,	 -0.194496478005509f,	-0.0604879757851760f,
//		1.0f,	2.55899172854618f,	5.01250988585749f,	 0.222353099471982f,	-0.0859879035317410f,
//		1.0f,	3.33330006334676f,	3.97375095736532f,	 -0.209546308715947f,	-0.100582416803848f,
//		1.0f,	2.08110251469923f,	4.04993418306817f,	 -0.201877789429608f,	-0.149965806195027f,
//		1.0f,	1.65878351982276f,	3.28762549526753f,	 -0.174258235551929f,	-0.207046828627029f,
//		1.0f,	1.29741765900866f,	2.58076808866115f,	 0.921773928707430f,	-0.221002008544677f,
//		1.0f,	1.94291925536370f,	0.966396475257705f,	 0.913003583863207f,	-0.242322339775031f,
//		1.0f,	1.03841829107468f,	1.99594189205706f,	 -0.129764463929572f,	-0.270267875503277f,
//		1.0f,	0.864162668608859f,	1.54650189787745f,	 0.897157074987238f,	-0.276418582719346f,
//		1.0f,	0.530564677078026f,	0.0708642029941283f, 0.871893661673881f,	-0.321404165938872f,
//		1.0f,	0.529883662399817f,	0.0746097785831167f, -0.0712411731258595f,	-0.338340339928839f,
//		1.0f,	0.528862242895311f,	0.0822371319226442f, 0.834862409610059f,	-0.374914766076250f,
//		1.0f,	1.06465876703604f,	0.626268882662256f,	 -0.000783778604352192f, -0.410271125270679f,
//		1.0f,	0.526915651661835f,	0.0938951232807263f, 0.785154116914545f,	-0.434615832262274f,
//		1.0f,	0.659658480864861f,	0.178609487365292f,	 0.0824975865782285f,	-0.484823717853870f,
//		1.0f,	0.523655054947558f,	0.110659667086024f,	 0.720995405259019f,	-0.497259438499837f,
//		1.0f,	0.742800224148955f,	1.20551049632142f,	 0.170303684134909f,	-0.544481262489368f,
//		1.0f,	0.517144851176093f,	0.135005711637948f,	 0.411891404238022f,	-0.553891873707613f,
//		1.0f,	0.502768796418184f,	0.167873810948848f,	 0.655875994436874f,	-0.560006388319740f,
//		1.0f,	0.491496642301725f,	0.213000553731805f,	 0.211126014935138f,	-0.622251270771263f,
//		1.0f,	0.491322562252740f,	0.276075577215757f,	 0.601184290034885f,	-0.654511228136602f,
//		1.0f,	0.508670575245701f,	0.362145410597963f,	 0.288152757500824f,	-0.734625075982280f,
//		1.0f,	0.553247431176431f,	0.471338002611028f,	 0.531918548859728f,	-0.767743113476969f,
//		1.0f,	0.590396739025035f,	0.575946113859356f,	 0.387145960218034f,	-0.842556438780516f,
//		1.0f,	0.654562367860189f,	0.936057748099734f,	 0.459200733978579f,	-0.871463775764269f,
//		1.0f,	0.602541907884154f,	0.723085131866804f,	 0.451939152463407f,	-0.955424332775676f,
};

const float32_t iir_df2T_coeff_f32[IIR_COEFF_COUNT] = {
	/* cutoff 10kHz, order 66
	 * b0	b1	b2	a1	a2  */
	0.081151347f,	0.162302695f,	0.081151347f,	0.416100782f,	-0.792953711f,
	0.177202897f,	0.354405794f,	0.177202897f,	0.358093443f,	-0.543003515f,
	0.243742079f,	0.487484157f,	0.243742079f,	0.316575117f,	-0.364103495f,
	0.289444386f,	0.578888773f,	0.289444386f,	0.286635666f,	-0.235096169f,
	0.319454787f,	0.638909574f,	0.319454787f,	0.265219174f,	-0.142813768f,
	0.336471322f,	0.672942644f,	0.336471322f,	0.250389528f,	-0.078913699f,
	0.340185363f,	0.680370727f,	0.340185363f,	0.240928402f,	-0.038146263f,
	0.301942282f,	0.603884564f,	0.301942282f,	0.23611087f,	-0.017387806f,
	0.194665316f,	0.389330633f,	0.194665316f,	0.235320385f,	-0.01398165f,
	0.196856394f,	0.393712788f,	0.196856394f,	0.23796906f,	-0.025394635f,
	0.202708174f,	0.405416347f,	0.202708174f,	0.24504296f,	-0.055875655f,
	0.212657948f,	0.425315897f,	0.212657948f,	0.257070705f,	-0.107702499f,
	0.227492558f,	0.454985116f,	0.227492558f,	0.275003464f,	-0.184973696f,
	0.24848634f,	0.49697268f,	0.24848634f,	0.300381713f,	-0.294327073f,
	0.277649477f,	0.555298955f,	0.277649477f,	0.335635454f,	-0.446233364f,
	0.318175645f,	0.63635129f,	0.318175645f,	0.384625349f,	-0.657327928f,
	0.187732516f,	0.375465032f,	0.187732516f,	0.453650093f,	-0.954751477f,
	0.108702874f,	0.217405747f,	0.108702874f,	0.399681548f,	-0.722204202f,
	0.198087926f,	0.396175852f,	0.198087926f,	0.34640137f,	-0.492623058f,
	0.262131914f,	0.524263829f,	0.262131914f,	0.308148545f,	-0.327793894f,
	0.308112958f,	0.616225916f,	0.308112958f,	0.280573359f,	-0.208974044f,
	0.340614139f,	0.681228278f,	0.340614139f,	0.260951915f,	-0.124426401f,
	0.362549402f,	0.725098804f,	0.362549402f,	0.247556479f,	-0.066706258f,
	0.375691113f,	0.751382226f,	0.375691113f,	0.239308211f,	-0.031164959f,
	0.236859377f,	0.473718755f,	0.236859377f,	0.23558349f,	-0.015115356f,
	0.195976063f,	0.391952127f,	0.195976063f,	0.236904876f,	-0.02080913f,
	0.200883361f,	0.401766722f,	0.200883361f,	0.242837043f,	-0.046370487f,
	0.209750263f,	0.419500527f,	0.209750263f,	0.253555762f,	-0.092556816f,
	0.223266419f,	0.446532838f,	0.223266419f,	0.269894712f,	-0.162960388f,
	0.242566376f,	0.485132752f,	0.242566376f,	0.293225388f,	-0.263490892f,
	0.269444337f,	0.538888674f,	0.269444337f,	0.325716703f,	-0.403494051f,
	0.306739667f,	0.613479334f,	0.306739667f,	0.370801013f,	-0.59775968f,
	0.359047384f,	0.718094768f,	0.359047384f,	0.434032986f,	-0.870222522f,
};

const float32_t iir_df2_coeff_f32[IIR_COEFF_COUNT] = {
	/* cutoff 10kHz, order 66
	 * a1	a2	b0	b1	b2	*/
	-0.416100782f,	0.792953711f,	0.035985196f,	0.071970392f,	0.035985196f,
	-0.358093443f,	0.543003515f,	0.24593792f,	0.491875841f,	0.24593792f,
	-0.316575117f,	0.364103495f,	0.292326008f,	0.584652016f,	0.292326008f,
	-0.286635666f,	0.235096169f,	0.323231352f,	0.646462704f,	0.323231352f,
	-0.265219174f,	0.142813768f,	0.341744243f,	0.683488487f,	0.341744243f,
	-0.250389528f,	0.078913699f,	0.349087413f,	0.698174825f,	0.349087413f,
	-0.240928402f,	0.038146263f,	0.342757509f,	0.685515018f,	0.342757509f,
	-0.23611087f,	0.017387806f,	0.255047221f,	0.510094441f,	0.255047221f,
	-0.235320385f,	0.01398165f,	0.196856394f,	0.393712788f,	0.196856394f,
	-0.23796906f,	0.025394635f,	0.202708174f,	0.405416347f,	0.202708174f,
	-0.24504296f,	0.055875655f,	0.212657948f,	0.425315897f,	0.212657948f,
	-0.257070705f,	0.107702499f,	0.227492558f,	0.454985116f,	0.227492558f,
	-0.275003464f,	0.184973696f,	0.24848634f,	0.49697268f,	0.24848634f,
	-0.300381713f,	0.294327073f,	0.277649477f,	0.555298955f,	0.277649477f,
	-0.335635454f,	0.446233364f,	0.318175645f,	0.63635129f,	0.318175645f,
	-0.384625349f,	0.657327928f,	0.116557064f,	0.233114129f,	0.116557064f,
	-0.453650093f,	0.954751477f,	0.108749869f,	0.217499739f,	0.108749869f,
	-0.399681548f,	0.722204202f,	0.198213273f,	0.396426545f,	0.198213273f,
	-0.34640137f,	0.492623058f,	0.262320034f,	0.524640069f,	0.262320034f,
	-0.308148545f,	0.327793894f,	0.308360525f,	0.61672105f,	0.308360525f,
	-0.280573359f,	0.208974044f,	0.340936401f,	0.681872801f,	0.340936401f,
	-0.260951915f,	0.124426401f,	0.3629516f,		0.725903201f,	0.3629516f,
	-0.247556479f,	0.066706258f,	0.376206256f,	0.752412511f,	0.376206256f,
	-0.239308211f,	0.031164959f,	0.367310471f,	0.734620942f,	0.367310471f,
	-0.23558349f,	0.015115356f,	0.202325709f,	0.404651417f,	0.202325709f,
	-0.236904876f,	0.02080913f,	0.200883361f,	0.401766722f,	0.200883361f,
	-0.242837043f,	0.046370487f,	0.209750263f,	0.419500527f,	0.209750263f,
	-0.253555762f,	0.092556816f,	0.223266419f,	0.446532838f,	0.223266419f,
	-0.269894712f,	0.162960388f,	0.242566376f,	0.485132752f,	0.242566376f,
	-0.293225388f,	0.263490892f,	0.269444337f,	0.538888674f,	0.269444337f,
	-0.325716703f,	0.403494051f,	0.306739667f,	0.613479334f,	0.306739667f,
	-0.370801013f,	0.59775968f,	0.2281868f,		0.4563736f,		0.2281868f,
	-0.434032986f,	0.870222522f,	0.39337002f,	0.786740039f,	0.39337002f,
};

#if !(Q31_USED)
arm_fir_instance_f32 fir_instance_f32_1;
arm_fir_instance_f32 fir_instance_f32_2;
float32_t fir_state_f32_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
float32_t fir_state_f32_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];

#if !(PQ_USED)
arm_biquad_casd_df1_inst_f32 iir_df1_instance_f32_1;
arm_biquad_casd_df1_inst_f32 iir_df1_instance_f32_2;
float32_t iir_df1_state_f32_1[IIR_SOS * 4];
float32_t iir_df1_state_f32_2[IIR_SOS * 4];

arm_biquad_cascade_df2T_instance_f32 iir_df2T_instance_f32_1;
arm_biquad_cascade_df2T_instance_f32 iir_df2T_instance_f32_2;
float32_t iir_df2T_state_f32_1[IIR_SOS * 2];
float32_t iir_df2T_state_f32_2[IIR_SOS * 2];
#else
pq_biquad_cascade_df2_instance iir_df2_instance_f32_1;
pq_biquad_cascade_df2_instance iir_df2_instance_f32_2;
float32_t iir_df2_state_f32_1[IIR_SOS * 8];
float32_t iir_df2_state_f32_2[IIR_SOS * 8];
#endif
#else
arm_fir_instance_q31 fir_instance_q31_1;
arm_fir_instance_q31 fir_instance_q31_2;
q31_t fir_state_q31_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
q31_t fir_state_q31_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
q31_t fir_coeff_q31[FIR_COEFF_COUNT];

arm_biquad_casd_df1_inst_q31 iir_df1_instance_q31_1;
arm_biquad_casd_df1_inst_q31 iir_df1_instance_q31_2;
q31_t iir_df1_state_q31_1[IIR_SOS * 4];
q31_t iir_df1_state_q31_2[IIR_SOS * 4];
q31_t iir_df1_coeff_q31[IIR_COEFF_COUNT];

pq_biquad_cascade_df2_instance iir_df2_instance_q31_1;
pq_biquad_cascade_df2_instance iir_df2_instance_q31_2;
q31_t iir_df2_state_q31_1[IIR_SOS * 8];
q31_t iir_df2_state_q31_2[IIR_SOS * 8];
//q31_t iir_df2_coeff_q31[IIR_COEFF_COUNT];
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void init_fir_filter(void)
{
	#if !(Q31_USED)
	arm_fir_init_f32(&fir_instance_f32_1, FIR_COEFF_COUNT, (float32_t *)&fir_coeff_f32[0], &fir_state_f32_1[0], FIR_BLOCK_SIZE);
	arm_fir_init_f32(&fir_instance_f32_2, FIR_COEFF_COUNT, (float32_t *)&fir_coeff_f32[0], &fir_state_f32_2[0], FIR_BLOCK_SIZE);
	#else
	arm_float_to_q31(fir_coeff_f32, fir_coeff_q31, FIR_COEFF_COUNT);
	arm_fir_init_q31(&fir_instance_q31_1, FIR_COEFF_COUNT, (q31_t *)&fir_coeff_q31[0], &fir_state_q31_1[0], FIR_BLOCK_SIZE);
	arm_fir_init_q31(&fir_instance_q31_2, FIR_COEFF_COUNT, (q31_t *)&fir_coeff_q31[0], &fir_state_q31_2[0], FIR_BLOCK_SIZE);
	#endif
}

#if !(PQ_USED)
void init_iir_df1_filter(void)
{
	#if !(Q31_USED)
	arm_biquad_cascade_df1_init_f32(&iir_df1_instance_f32_1, IIR_SOS, (float32_t *)iir_df1_coeff_f32, iir_df1_state_f32_1);
	arm_biquad_cascade_df1_init_f32(&iir_df1_instance_f32_2, IIR_SOS, (float32_t *)iir_df1_coeff_f32, iir_df1_state_f32_2);
	#else
	arm_float_to_q31(iir_df1_coeff_f32, iir_df1_coeff_q31, IIR_COEFF_COUNT); //???
	arm_biquad_cascade_df1_init_q31(&iir_df1_instance_q31_1, IIR_SOS, (q31_t *)iir_df1_coeff_q31, iir_df1_state_q31_1, IIR_Q31_POSTSHIFT);
	arm_biquad_cascade_df1_init_q31(&iir_df1_instance_q31_2, IIR_SOS, (q31_t *)iir_df1_coeff_q31, iir_df1_state_q31_2, IIR_Q31_POSTSHIFT);
	#endif
}
#endif

#if !(Q31_USED)
#if !(PQ_USED)
void init_iir_df2T_filter(void)
{
	arm_biquad_cascade_df2T_init_f32(&iir_df2T_instance_f32_1, IIR_SOS, (float32_t *)iir_df2T_coeff_f32, iir_df2T_state_f32_1);
	arm_biquad_cascade_df2T_init_f32(&iir_df2T_instance_f32_2, IIR_SOS, (float32_t *)iir_df2T_coeff_f32, iir_df2T_state_f32_2);
}
#endif
#endif

#if !(Q31_USED)
#if PQ_USED
void init_iir_df2_filter(void)
{
	arm_fill_f32(0.0f, iir_df2_state_f32_1, IIR_SOS * 8);
	for (int i = 0; i < IIR_SOS; ++i)
	{
		arm_copy_f32(&(iir_df2_coeff_f32[i * 5]), &(iir_df2_state_f32_1[(i * 8) + 2]), 5U);
//		iir_df2_state_f32_1[(i * 8) + 2] = iir_df2_coeff_f32[(i * 5) + 0];
//		iir_df2_state_f32_1[(i * 8) + 3] = iir_df2_coeff_f32[(i * 5) + 1];
//		iir_df2_state_f32_1[(i * 8) + 4] = iir_df2_coeff_f32[(i * 5) + 2];
//		iir_df2_state_f32_1[(i * 8) + 5] = iir_df2_coeff_f32[(i * 5) + 3];
//		iir_df2_state_f32_1[(i * 8) + 6] = iir_df2_coeff_f32[(i * 5) + 4];
	}
	PQ_BiquadCascadeDf2Init(&iir_df2_instance_f32_1, IIR_SOS, (pq_biquad_state_t *)iir_df2_state_f32_1);

	arm_fill_f32(0.0f, iir_df2_state_f32_2, IIR_SOS * 8);
	for (int i = 0; i < IIR_SOS; ++i)
	{
		arm_copy_f32(&(iir_df2_coeff_f32[i * 5]), &(iir_df2_state_f32_2[(i * 8) + 2]), 5U);
//		iir_df2_state_f32_2[(i * 8) + 2] = iir_df2_coeff_f32[(i * 5) + 0];
//		iir_df2_state_f32_2[(i * 8) + 3] = iir_df2_coeff_f32[(i * 5) + 1];
//		iir_df2_state_f32_2[(i * 8) + 4] = iir_df2_coeff_f32[(i * 5) + 2];
//		iir_df2_state_f32_2[(i * 8) + 5] = iir_df2_coeff_f32[(i * 5) + 3];
//		iir_df2_state_f32_2[(i * 8) + 6] = iir_df2_coeff_f32[(i * 5) + 4];
	}
	PQ_BiquadCascadeDf2Init(&iir_df2_instance_f32_2, IIR_SOS, (pq_biquad_state_t *)iir_df2_state_f32_2);
}
#endif
#else
void init_iir_df2_filter(void)
{
//	arm_float_to_q31(iir_df2_coeff_f32, iir_df2_coeff_q31, IIR_COEFF_COUNT); //???

	arm_fill_q31(Q31_ZERO, iir_df2_state_q31_1, IIR_SOS * 8);
	for (int i = 0; i < IIR_SOS; ++i)
	{
		arm_copy_f32(&(iir_df2_coeff_f32[i * 5]), (float32_t *)&(iir_df2_state_q31_1[(i * 8) + 2]), 5U);
	}
	PQ_BiquadCascadeDf2Init(&iir_df2_instance_q31_1, IIR_SOS, (pq_biquad_state_t *)iir_df2_state_q31_1);

	arm_fill_q31(Q31_ZERO, iir_df2_state_q31_2, IIR_SOS * 8);
	for (int i = 0; i < IIR_SOS; ++i)
	{
		arm_copy_f32(&(iir_df2_coeff_f32[i * 5]), (float32_t *)&(iir_df2_state_q31_2[(i * 8) + 2]), 5U);
	}
	PQ_BiquadCascadeDf2Init(&iir_df2_instance_q31_2, IIR_SOS, (pq_biquad_state_t *)iir_df2_state_q31_2);
}
#endif

#if !(Q31_USED)
void fir_process_batch(float32_t * src_buffer, float32_t * dst_buffer)
{
	#if !(PQ_USED)
	for (uint32_t j = 0; j < fir_num_blocks; ++j)
	{
		arm_fir_f32(&fir_instance_f32_1, &src_buffer[0] + (j * FIR_BLOCK_SIZE), &dst_buffer[0] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
	}
	for (uint32_t j = 0; j < fir_num_blocks; ++j)
	{
		arm_fir_f32(&fir_instance_f32_2, &src_buffer[BUFFER_SIZE/2] + (j * FIR_BLOCK_SIZE), &dst_buffer[BUFFER_SIZE/2] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
	}
	#else
	const static int old_val_idx[CHANNEL_CNT] = {0, (FIR_COEFF_COUNT-1) + (BUFFER_SIZE/2)};
	const static int new_val_idx[CHANNEL_CNT] = {(FIR_COEFF_COUNT-1), ((2*(FIR_COEFF_COUNT-1)) + (BUFFER_SIZE/2))};
	static bool is_first_call = true;
	int offset;

	/* copy the new samples to the the src_state buffer */
	arm_copy_f32(&src_buffer[0], &src_state_buffer[new_val_idx[CHANNEL_LEFT]], (BUFFER_SIZE/2));
	arm_copy_f32(&src_buffer[BUFFER_SIZE/2], &src_state_buffer[new_val_idx[CHANNEL_RIGHT]], (BUFFER_SIZE/2));

	if (is_first_call)
	{
		PQ_FIR(POWERQUAD, &src_state_buffer[old_val_idx[CHANNEL_CNT]], (int32_t)FIR_BLOCK_SIZE, &fir_instance_f32_1.pState[1], FIR_COEFF_COUNT, &dst_state_buffer[old_val_idx[CHANNEL_CNT]], PQ_FIR_FIR);
		PQ_WaitDone(POWERQUAD);
		is_first_call = false;
	}
	for (uint32_t i = 0; i < fir_num_blocks; ++i)
	{
		offset = i * FIR_BLOCK_SIZE + new_val_idx[CHANNEL_LEFT];
		PQ_FIRIncrement(POWERQUAD, (int32_t)FIR_BLOCK_SIZE, FIR_COEFF_COUNT, offset);
		PQ_WaitDone(POWERQUAD);
	}

	for (uint32_t j = 0; j < fir_num_blocks; ++j)
	{
		offset = j * FIR_BLOCK_SIZE + new_val_idx[CHANNEL_RIGHT];
		PQ_FIRIncrement(POWERQUAD, FIR_BLOCK_SIZE, FIR_COEFF_COUNT, offset);
		PQ_WaitDone(POWERQUAD);
	}

	/* copy the last numTaps - 1 samples to the state buffer */
	arm_copy_f32(&dst_state_buffer[new_val_idx[CHANNEL_LEFT]], &dst_buffer[0], (BUFFER_SIZE/2));
	arm_copy_f32(&dst_state_buffer[new_val_idx[CHANNEL_RIGHT]], &dst_buffer[BUFFER_SIZE/2], (BUFFER_SIZE/2));

	/* copy the last numTaps - 1 samples to the src_state buffer */
	arm_copy_f32(&src_state_buffer[new_val_idx[CHANNEL_LEFT]], &src_state_buffer[old_val_idx[CHANNEL_LEFT]], (FIR_COEFF_COUNT-1));
	arm_copy_f32(&src_state_buffer[new_val_idx[CHANNEL_RIGHT]], &src_state_buffer[old_val_idx[CHANNEL_RIGHT]], (FIR_COEFF_COUNT-1));
	#endif /* PQ_USED */
}
#else
void fir_process_batch(q31_t * src_buffer, q31_t * dst_buffer)
{
	#if !(PQ_USED)
	for (uint32_t j = 0; j < fir_num_blocks; ++j)
	{
		arm_fir_q31(&fir_instance_q31_1, &src_buffer[0] + (j * FIR_BLOCK_SIZE), &dst_buffer[0] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
		arm_fir_q31(&fir_instance_q31_2, &src_buffer[BUFFER_SIZE/2] + (j * FIR_BLOCK_SIZE), &dst_buffer[BUFFER_SIZE/2] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
	}
	#else
	const static int old_val_idx[CHANNEL_CNT] = {0, (FIR_COEFF_COUNT-1) + (BUFFER_SIZE/2)};
	const static int new_val_idx[CHANNEL_CNT] = {(FIR_COEFF_COUNT-1), ((2*(FIR_COEFF_COUNT-1)) + (BUFFER_SIZE/2))};
	static bool is_first_call = true;
	int offset;

	/* copy the new samples to the the src_state buffer */
	arm_copy_q31(&src_buffer[0], &src_state_buffer[new_val_idx[CHANNEL_LEFT]], (BUFFER_SIZE/2));
	arm_copy_q31(&src_buffer[BUFFER_SIZE/2], &src_state_buffer[new_val_idx[CHANNEL_RIGHT]], (BUFFER_SIZE/2));

	if (is_first_call)
	{
		PQ_FIR(POWERQUAD, &src_state_buffer[old_val_idx[CHANNEL_CNT]], (int32_t)FIR_BLOCK_SIZE, &fir_instance_q31_1.pState[1], FIR_COEFF_COUNT, &dst_state_buffer[old_val_idx[CHANNEL_CNT]], PQ_FIR_FIR);
		PQ_WaitDone(POWERQUAD);
		is_first_call = false;
	}
	for (uint32_t i = 0; i < fir_num_blocks; ++i)
	{
		offset = i * FIR_BLOCK_SIZE + new_val_idx[CHANNEL_LEFT];
		PQ_FIRIncrement(POWERQUAD, (int32_t)FIR_BLOCK_SIZE, FIR_COEFF_COUNT, offset);
		PQ_WaitDone(POWERQUAD);
	}

	for (uint32_t j = 0; j < fir_num_blocks; ++j)
	{
		offset = j * FIR_BLOCK_SIZE + new_val_idx[CHANNEL_RIGHT];
		PQ_FIRIncrement(POWERQUAD, FIR_BLOCK_SIZE, FIR_COEFF_COUNT, offset);
		PQ_WaitDone(POWERQUAD);
	}

	/* copy the last numTaps - 1 samples to the state buffer */
	arm_copy_q31(&dst_state_buffer[new_val_idx[CHANNEL_LEFT]], &dst_buffer[0], (BUFFER_SIZE/2));
	arm_copy_q31(&dst_state_buffer[new_val_idx[CHANNEL_RIGHT]], &dst_buffer[BUFFER_SIZE/2], (BUFFER_SIZE/2));

	/* copy the last numTaps - 1 samples to the src_state buffer */
	arm_copy_q31(&src_state_buffer[new_val_idx[CHANNEL_LEFT]], &src_state_buffer[old_val_idx[CHANNEL_LEFT]], (FIR_COEFF_COUNT-1));
	arm_copy_q31(&src_state_buffer[new_val_idx[CHANNEL_RIGHT]], &src_state_buffer[old_val_idx[CHANNEL_RIGHT]], (FIR_COEFF_COUNT-1));
	#endif /* PQ_USED */
}
#endif

#if !(PQ_USED)
#if !(Q31_USED)
void iir_df1_process_batch(float32_t * src_buffer, float32_t * dst_buffer)
{
	arm_biquad_cascade_df1_f32(&iir_df1_instance_f32_1, &src_buffer[0], &dst_buffer[0], BUFFER_SIZE/2);
	arm_biquad_cascade_df1_f32(&iir_df1_instance_f32_2, &src_buffer[BUFFER_SIZE/2], &dst_buffer[BUFFER_SIZE/2], BUFFER_SIZE/2);
}
#else
void iir_df1_process_batch(q31_t * src_buffer, q31_t * dst_buffer)
{
	arm_biquad_cascade_df1_q31(&iir_df1_instance_q31_1, &src_buffer[0], &dst_buffer[0], BUFFER_SIZE/2);
	arm_biquad_cascade_df1_q31(&iir_df1_instance_q31_2, &src_buffer[BUFFER_SIZE/2], &dst_buffer[BUFFER_SIZE/2], BUFFER_SIZE/2);
}
#endif
#endif

#if !(Q31_USED)
#if !(PQ_USED)
void iir_df2T_process_batch(float32_t * src_buffer, float32_t * dst_buffer)
{
	arm_biquad_cascade_df2_f32(&iir_df2T_instance_f32_1, &src_buffer[0], &dst_buffer[0], BUFFER_SIZE/2);
	arm_biquad_cascade_df2_f32(&iir_df2T_instance_f32_2, &src_buffer[BUFFER_SIZE/2], &dst_buffer[BUFFER_SIZE/2], BUFFER_SIZE/2);
}
#endif
#endif

#if PQ_USED
#if !(Q31_USED)
void iir_df2_process_batch(float32_t * src_buffer, float32_t * dst_buffer)
{
	PQ_BiquadCascadeDf2F32(&iir_df2_instance_f32_1, &src_buffer[0], &dst_buffer[0], BUFFER_SIZE/2);
	PQ_BiquadCascadeDf2F32(&iir_df2_instance_f32_2, &src_buffer[BUFFER_SIZE/2], &dst_buffer[BUFFER_SIZE/2], BUFFER_SIZE/2);
}
#else
void iir_df2_process_batch(q31_t * src_buffer, q31_t * dst_buffer)
{
	PQ_BiquadCascadeDf2Fixed32(&iir_df2_instance_q31_1, &src_buffer[0], &dst_buffer[0], BUFFER_SIZE/2);
	PQ_BiquadCascadeDf2Fixed32(&iir_df2_instance_q31_2, &src_buffer[BUFFER_SIZE/2], &dst_buffer[BUFFER_SIZE/2], BUFFER_SIZE/2);
}
#endif
#endif
