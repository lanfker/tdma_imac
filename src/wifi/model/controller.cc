/*
 * created by Chuan
 * this fine mainly implements various controllers for the PRK model.
 *
 */


#include "controller.h"
#include "ns3/log.h"
#include "math.h"

NS_LOG_COMPONENT_DEFINE ("Controller");
namespace ns3
{

  NS_OBJECT_ENSURE_REGISTERED (Controller);
  TypeId Controller::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Controller")
      .SetParent<Object> ()
      .AddConstructor <Controller> ()
      ;
    return tid;
  }
  Controller::Controller ()
  {
    NS_LOG_FUNCTION (this);
  }
  Controller::~Controller ()
  {
    NS_LOG_FUNCTION (this);
  }
  /*
   

  const double Controller::m_pdrToSnr[1000] = {// second version
    0.000000, 1.295600, 1.380200, 1.433900, 1.474500, 1.507400, 1.535500, 1.560000, 1.582000, 1.601900, 
    1.620100, 1.637000, 1.652800, 1.667600, 1.681500, 1.694800, 1.707400, 1.719400, 1.730900, 1.742000, 
    1.752600, 1.762900, 1.772800, 1.782400, 1.791700, 1.800700, 1.809500, 1.818000, 1.826300, 1.834400, 
    1.842300, 1.850100, 1.857600, 1.865000, 1.872300, 1.879400, 1.886400, 1.893200, 1.899900, 1.906500, 
    1.913000, 1.919400, 1.925700, 1.931800, 1.937900, 1.943900, 1.949800, 1.955600, 1.961400, 1.967000, 
    1.972600, 1.978200, 1.983600, 1.989000, 1.994300, 1.999600, 2.004800, 2.009900, 2.015000, 2.020000, 
    2.025000, 2.029900, 2.034800, 2.039600, 2.044400, 2.049100, 2.053800, 2.058400, 2.063000, 2.067600, 
    2.072100, 2.076600, 2.081000, 2.085400, 2.089800, 2.094200, 2.098500, 2.102700, 2.107000, 2.111200, 
    2.115300, 2.119500, 2.123600, 2.127700, 2.131800, 2.135800, 2.139800, 2.143800, 2.147700, 2.151600, 
    2.155500, 2.159400, 2.163300, 2.167100, 2.170900, 2.174700, 2.178500, 2.182200, 2.185900, 2.189600, 
    2.193300, 2.197000, 2.200600, 2.204200, 2.207800, 2.211400, 2.215000, 2.218500, 2.222000, 2.225600, 
    2.229100, 2.232500, 2.236000, 2.239400, 2.242900, 2.246300, 2.249700, 2.253100, 2.256400, 2.259800, 
    2.263100, 2.266500, 2.269800, 2.273100, 2.276400, 2.279600, 2.282900, 2.286200, 2.289400, 2.292600, 
    2.295800, 2.299000, 2.302200, 2.305400, 2.308600, 2.311700, 2.314900, 2.318000, 2.321100, 2.324200, 
    2.327300, 2.330400, 2.333500, 2.336600, 2.339600, 2.342700, 2.345700, 2.348700, 2.351800, 2.354800, 
    2.357800, 2.360800, 2.363800, 2.366700, 2.369700, 2.372700, 2.375600, 2.378600, 2.381500, 2.384400, 
    2.387300, 2.390200, 2.393200, 2.396000, 2.398900, 2.401800, 2.404700, 2.407600, 2.410400, 2.413300, 
    2.416100, 2.419000, 2.421800, 2.424600, 2.427400, 2.430200, 2.433000, 2.435800, 2.438600, 2.441400, 
    2.444200, 2.447000, 2.449800, 2.452500, 2.455300, 2.458000, 2.460800, 2.463500, 2.466200, 2.469000, 
    2.471700, 2.474400, 2.477100, 2.479800, 2.482500, 2.485200, 2.487900, 2.490600, 2.493300, 2.496000, 
    2.498700, 2.501300, 2.504000, 2.506700, 2.509300, 2.512000, 2.514600, 2.517200, 2.519900, 2.522500, 
    2.525100, 2.527800, 2.530400, 2.533000, 2.535600, 2.538200, 2.540800, 2.543400, 2.546000, 2.548600, 
    2.551200, 2.553800, 2.556400, 2.559000, 2.561500, 2.564100, 2.566700, 2.569300, 2.571800, 2.574400, 
    2.576900, 2.579500, 2.582000, 2.584600, 2.587100, 2.589700, 2.592200, 2.594700, 2.597300, 2.599800, 
    2.602300, 2.604800, 2.607400, 2.609900, 2.612400, 2.614900, 2.617400, 2.619900, 2.622400, 2.624900, 
    2.627400, 2.629900, 2.632400, 2.634900, 2.637400, 2.639900, 2.642400, 2.644800, 2.647300, 2.649800, 
    2.652300, 2.654700, 2.657200, 2.659700, 2.662100, 2.664600, 2.667100, 2.669500, 2.672000, 2.674400, 
    2.676900, 2.679400, 2.681800, 2.684300, 2.686700, 2.689100, 2.691600, 2.694000, 2.696500, 2.698900, 
    2.701300, 2.703800, 2.706200, 2.708600, 2.711100, 2.713500, 2.715900, 2.718400, 2.720800, 2.723200, 
    2.725600, 2.728100, 2.730500, 2.732900, 2.735300, 2.737700, 2.740100, 2.742600, 2.745000, 2.747400, 
    2.749800, 2.752200, 2.754600, 2.757000, 2.759400, 2.761800, 2.764200, 2.766600, 2.769000, 2.771400, 
    2.773800, 2.776200, 2.778600, 2.781000, 2.783400, 2.785800, 2.788200, 2.790600, 2.793000, 2.795400, 
    2.797800, 2.800200, 2.802600, 2.804900, 2.807300, 2.809700, 2.812100, 2.814500, 2.816900, 2.819300, 
    2.821700, 2.824000, 2.826400, 2.828800, 2.831200, 2.833600, 2.836000, 2.838300, 2.840700, 2.843100, 
    2.845500, 2.847900, 2.850300, 2.852600, 2.855000, 2.857400, 2.859800, 2.862200, 2.864500, 2.866900, 
    2.869300, 2.871700, 2.874100, 2.876400, 2.878800, 2.881200, 2.883600, 2.886000, 2.888300, 2.890700, 
    2.893100, 2.895500, 2.897900, 2.900200, 2.902600, 2.905000, 2.907400, 2.909800, 2.912100, 2.914500, 
    2.916900, 2.919300, 2.921700, 2.924100, 2.926400, 2.928800, 2.931200, 2.933600, 2.936000, 2.938400, 
    2.940700, 2.943100, 2.945500, 2.947900, 2.950300, 2.952700, 2.955100, 2.957500, 2.959800, 2.962200, 
    2.964600, 2.967000, 2.969400, 2.971800, 2.974200, 2.976600, 2.979000, 2.981400, 2.983800, 2.986200, 
    2.988600, 2.991000, 2.993400, 2.995800, 2.998200, 3.000600, 3.003000, 3.005400, 3.007800, 3.010200, 
    3.012600, 3.015000, 3.017400, 3.019800, 3.022200, 3.024600, 3.027000, 3.029400, 3.031800, 3.034300, 
    3.036700, 3.039100, 3.041500, 3.043900, 3.046300, 3.048800, 3.051200, 3.053600, 3.056000, 3.058500, 
    3.060900, 3.063300, 3.065700, 3.068200, 3.070600, 3.073000, 3.075500, 3.077900, 3.080300, 3.082800, 
    3.085200, 3.087700, 3.090100, 3.092600, 3.095000, 3.097400, 3.099900, 3.102300, 3.104800, 3.107300, 
    3.109700, 3.112200, 3.114600, 3.117100, 3.119500, 3.122000, 3.124500, 3.126900, 3.129400, 3.131900, 
    3.134300, 3.136800, 3.139300, 3.141800, 3.144300, 3.146700, 3.149200, 3.151700, 3.154200, 3.156700, 
    3.159200, 3.161700, 3.164100, 3.166600, 3.169100, 3.171600, 3.174100, 3.176600, 3.179200, 3.181700, 
    3.184200, 3.186700, 3.189200, 3.191700, 3.194200, 3.196800, 3.199300, 3.201800, 3.204300, 3.206900, 
    3.209400, 3.211900, 3.214500, 3.217000, 3.219500, 3.222100, 3.224600, 3.227200, 3.229700, 3.232300, 
    3.234800, 3.237400, 3.240000, 3.242500, 3.245100, 3.247700, 3.250200, 3.252800, 3.255400, 3.257900, 
    3.260500, 3.263100, 3.265700, 3.268300, 3.270900, 3.273500, 3.276100, 3.278700, 3.281300, 3.283900, 
    3.286500, 3.289100, 3.291700, 3.294300, 3.297000, 3.299600, 3.302200, 3.304800, 3.307500, 3.310100, 
    3.312700, 3.315400, 3.318000, 3.320700, 3.323300, 3.326000, 3.328600, 3.331300, 3.334000, 3.336600, 
    3.339300, 3.342000, 3.344600, 3.347300, 3.350000, 3.352700, 3.355400, 3.358100, 3.360800, 3.363500, 
    3.366200, 3.368900, 3.371600, 3.374300, 3.377000, 3.379800, 3.382500, 3.385200, 3.387900, 3.390700, 
    3.393400, 3.396200, 3.398900, 3.401700, 3.404400, 3.407200, 3.409900, 3.412700, 3.415500, 3.418300, 
    3.421000, 3.423800, 3.426600, 3.429400, 3.432200, 3.435000, 3.437800, 3.440600, 3.443400, 3.446200, 
    3.449100, 3.451900, 3.454700, 3.457500, 3.460400, 3.463200, 3.466100, 3.468900, 3.471800, 3.474600, 
    3.477500, 3.480400, 3.483200, 3.486100, 3.489000, 3.491900, 3.494800, 3.497700, 3.500600, 3.503500, 
    3.506400, 3.509300, 3.512300, 3.515200, 3.518100, 3.521000, 3.524000, 3.526900, 3.529900, 3.532800, 
    3.535800, 3.538800, 3.541700, 3.544700, 3.547700, 3.550700, 3.553700, 3.556700, 3.559700, 3.562700, 
    3.565700, 3.568700, 3.571800, 3.574800, 3.577800, 3.580900, 3.583900, 3.587000, 3.590100, 3.593100, 
    3.596200, 3.599300, 3.602400, 3.605500, 3.608600, 3.611700, 3.614800, 3.617900, 3.621000, 3.624100, 
    3.627300, 3.630400, 3.633600, 3.636700, 3.639900, 3.643000, 3.646200, 3.649400, 3.652600, 3.655800, 
    3.659000, 3.662200, 3.665400, 3.668600, 3.671800, 3.675100, 3.678300, 3.681600, 3.684800, 3.688100, 
    3.691300, 3.694600, 3.697900, 3.701200, 3.704500, 3.707800, 3.711100, 3.714400, 3.717800, 3.721100, 
    3.724400, 3.727800, 3.731200, 3.734500, 3.737900, 3.741300, 3.744700, 3.748100, 3.751500, 3.754900, 
    3.758300, 3.761800, 3.765200, 3.768700, 3.772100, 3.775600, 3.779100, 3.782500, 3.786000, 3.789500, 
    3.793000, 3.796600, 3.800100, 3.803600, 3.807200, 3.810700, 3.814300, 3.817900, 3.821500, 3.825000, 
    3.828600, 3.832300, 3.835900, 3.839500, 3.843200, 3.846800, 3.850500, 3.854100, 3.857800, 3.861500, 
    3.865200, 3.868900, 3.872600, 3.876400, 3.880100, 3.883900, 3.887600, 3.891400, 3.895200, 3.899000, 
    3.902800, 3.906600, 3.910500, 3.914300, 3.918200, 3.922000, 3.925900, 3.929800, 3.933700, 3.937600, 
    3.941500, 3.945500, 3.949400, 3.953400, 3.957400, 3.961300, 3.965300, 3.969300, 3.973400, 3.977400, 
    3.981500, 3.985500, 3.989600, 3.993700, 3.997800, 4.001900, 4.006000, 4.010200, 4.014300, 4.018500, 
    4.022700, 4.026900, 4.031100, 4.035300, 4.039600, 4.043800, 4.048100, 4.052400, 4.056700, 4.061000, 
    4.065300, 4.069700, 4.074100, 4.078400, 4.082800, 4.087200, 4.091700, 4.096100, 4.100600, 4.105100, 
    4.109600, 4.114100, 4.118600, 4.123100, 4.127700, 4.132300, 4.136900, 4.141500, 4.146100, 4.150800, 
    4.155500, 4.160100, 4.164900, 4.169600, 4.174300, 4.179100, 4.183900, 4.188700, 4.193500, 4.198400, 
    4.203200, 4.208100, 4.213000, 4.217900, 4.222900, 4.227900, 4.232900, 4.237900, 4.242900, 4.248000, 
    4.253000, 4.258100, 4.263300, 4.268400, 4.273600, 4.278800, 4.284000, 4.289200, 4.294500, 4.299800, 
    4.305100, 4.310500, 4.315800, 4.321200, 4.326600, 4.332100, 4.337600, 4.343100, 4.348600, 4.354100, 
    4.359700, 4.365300, 4.371000, 4.376600, 4.382300, 4.388100, 4.393800, 4.399600, 4.405400, 4.411300, 
    4.417100, 4.423100, 4.429000, 4.435000, 4.441000, 4.447000, 4.453100, 4.459200, 4.465300, 4.471500, 
    4.477700, 4.484000, 4.490300, 4.496600, 4.503000, 4.509300, 4.515800, 4.522300, 4.528800, 4.535300, 
    4.541900, 4.548500, 4.555200, 4.561900, 4.568700, 4.575500, 4.582300, 4.589200, 4.596200, 4.603200, 
    4.610200, 4.617300, 4.624400, 4.631600, 4.638800, 4.646000, 4.653400, 4.660700, 4.668200, 4.675600, 
    4.683200, 4.690700, 4.698400, 4.706100, 4.713800, 4.721700, 4.729500, 4.737500, 4.745400, 4.753500, 
    4.761600, 4.769800, 4.778000, 4.786400, 4.794700, 4.803200, 4.811700, 4.820300, 4.829000, 4.837700, 
    4.846500, 4.855400, 4.864400, 4.873400, 4.882500, 4.891800, 4.901100, 4.910400, 4.919900, 4.929500, 
    4.939100, 4.948800, 4.958700, 4.968600, 4.978600, 4.988800, 4.999000, 5.009300, 5.019800, 5.030300, 
    5.041000, 5.051800, 5.062700, 5.073700, 5.084900, 5.096100, 5.107500, 5.119100, 5.130700, 5.142500, 
    5.154500, 5.166600, 5.178800, 5.191200, 5.203700, 5.216500, 5.229300, 5.242400, 5.255600, 5.269000, 
    5.282600, 5.296400, 5.310300, 5.324500, 5.338900, 5.353500, 5.368300, 5.383400, 5.398600, 5.414200, 
    5.429900, 5.446000, 5.462300, 5.478900, 5.495700, 5.512900, 5.530400, 5.548200, 5.566300, 5.584800, 
    5.603600, 5.622800, 5.642400, 5.662400, 5.682800, 5.703700, 5.725000, 5.746900, 5.769200, 5.792000, 
    5.815400, 5.839400, 5.864100, 5.889300, 5.915300, 5.941900, 5.969400, 5.997600, 6.026700, 6.056700, 
    6.087800, 6.119800, 6.153000, 6.187300, 6.223000, 6.260000, 6.298500, 6.338700, 6.380600, 6.424500, 
    6.470600, 6.518900, 6.569900, 6.623800, 6.680900, 6.741700, 6.806700, 6.876500, 6.951900, 7.033800, 
    7.123500, 7.222600, 7.333400, 7.459000, 7.603800, 7.774900, 7.984200, 8.253300, 8.630800, 9.267900
  };

  const double Controller::m_pdrToSnr[1000] = { // first veresion

    1.124600, 1.399200, 1.565200, 1.686200, 1.782300, 1.862400, 1.931300, 1.991900, 2.046200, 2.095400, 2.140400, 2.182100, 2.220800, 2.257000, 2.291100,
    2.323200, 2.353700, 2.382700, 2.410300, 2.436800, 2.462100, 2.486400, 2.509900, 2.532500, 2.554300, 2.575400, 2.595800, 2.615700, 2.634900, 2.653600,
    2.671800, 2.689500, 2.706800, 2.723700, 2.740100, 2.756200, 2.771900, 2.787300, 2.802400, 2.817100, 2.831600, 2.845700, 2.859700, 2.873300, 2.886700,
    2.899900, 2.912800, 2.925600, 2.938100, 2.950400, 2.962500, 2.974500, 2.986300, 2.997900, 3.009300, 3.020600, 3.031700, 3.042600, 3.053500, 3.064100,
    3.074700, 3.085100, 3.095400, 3.105500, 3.115600, 3.125500, 3.135300, 3.145000, 3.154600, 3.164100, 3.173500, 3.182700, 3.191900, 3.201000, 3.210000,
    3.219000, 3.227800, 3.236500, 3.245200, 3.253800, 3.262300, 3.270700, 3.279000, 3.287300, 3.295500, 3.303700, 3.311700, 3.319700, 3.327600, 3.335500,
    3.343300, 3.351100, 3.358700, 3.366400, 3.373900, 3.381400, 3.388900, 3.396300, 3.403600, 3.410900, 3.418200, 3.425400, 3.432500, 3.439600, 3.446600,
    3.453600, 3.460600, 3.467500, 3.474300, 3.481200, 3.487900, 3.494700, 3.501300, 3.508000, 3.514600, 3.521200, 3.527700, 3.534200, 3.540700, 3.547100,
    3.553500, 3.559800, 3.566100, 3.572400, 3.578600, 3.584800, 3.591000, 3.597200, 3.603300, 3.609400, 3.615400, 3.621400, 3.627400, 3.633400, 3.639300,
    3.645200, 3.651100, 3.656900, 3.662700, 3.668500, 3.674300, 3.680000, 3.685700, 3.691400, 3.697100, 3.702700, 3.708300, 3.713900, 3.719500, 3.725000,
    3.730500, 3.736000, 3.741500, 3.746900, 3.752300, 3.757700, 3.763100, 3.768400, 3.773800, 3.779100, 3.784400, 3.789700, 3.794900, 3.800100, 3.805400,
    3.810600, 3.815700, 3.820900, 3.826000, 3.831100, 3.836200, 3.841300, 3.846400, 3.851400, 3.856500, 3.861500, 3.866500, 3.871400, 3.876400, 3.881300,
    3.886300, 3.891200, 3.896100, 3.901000, 3.905800, 3.910700, 3.915500, 3.920300, 3.925100, 3.929900, 3.934700, 3.939500, 3.944200, 3.948900, 3.953600,
    3.958400, 3.963000, 3.967700, 3.972400, 3.977000, 3.981700, 3.986300, 3.990900, 3.995500, 4.000100, 4.004700, 4.009200, 4.013800, 4.018300, 4.022800,
    4.027300, 4.031800, 4.036300, 4.040800, 4.045300, 4.049700, 4.054200, 4.058600, 4.063000, 4.067400, 4.071800, 4.076200, 4.080600, 4.085000, 4.089300,
    4.093700, 4.098000, 4.102400, 4.106700, 4.111000, 4.115300, 4.119600, 4.123900, 4.128100, 4.132400, 4.136600, 4.140900, 4.145100, 4.149300, 4.153600,
    4.157800, 4.162000, 4.166200, 4.170300, 4.174500, 4.178700, 4.182800, 4.187000, 4.191100, 4.195300, 4.199400, 4.203500, 4.207600, 4.211700, 4.215800,
    4.219900, 4.224000, 4.228000, 4.232100, 4.236100, 4.240200, 4.244200, 4.248300, 4.252300, 4.256300, 4.260300, 4.264300, 4.268300, 4.272300, 4.276300,
    4.280300, 4.284200, 4.288200, 4.292200, 4.296100, 4.300100, 4.304000, 4.307900, 4.311900, 4.315800, 4.319700, 4.323600, 4.327500, 4.331400, 4.335300,
    4.339200, 4.343000, 4.346900, 4.350800, 4.354600, 4.358500, 4.362300, 4.366200, 4.370000, 4.373900, 4.377700, 4.381500, 4.385300, 4.389100, 4.393000,
    4.396800, 4.400600, 4.404300, 4.408100, 4.411900, 4.415700, 4.419500, 4.423200, 4.427000, 4.430800, 4.434500, 4.438300, 4.442000, 4.445700, 4.449500,
    4.453200, 4.456900, 4.460700, 4.464400, 4.468100, 4.471800, 4.475500, 4.479200, 4.482900, 4.486600, 4.490300, 4.494000, 4.497700, 4.501300, 4.505000,
    4.508700, 4.512400, 4.516000, 4.519700, 4.523300, 4.527000, 4.530600, 4.534300, 4.537900, 4.541600, 4.545200, 4.548800, 4.552400, 4.556100, 4.559700,
    4.563300, 4.566900, 4.570500, 4.574100, 4.577700, 4.581300, 4.584900, 4.588500, 4.592100, 4.595700, 4.599300, 4.602900, 4.606500, 4.610000, 4.613600,
    4.617200, 4.620800, 4.624300, 4.627900, 4.631400, 4.635000, 4.638600, 4.642100, 4.645700, 4.649200, 4.652800, 4.656300, 4.659800, 4.663400, 4.666900,
    4.670400, 4.674000, 4.677500, 4.681000, 4.684600, 4.688100, 4.691600, 4.695100, 4.698600, 4.702100, 4.705600, 4.709200, 4.712700, 4.716200, 4.719700,
    4.723200, 4.726700, 4.730200, 4.733700, 4.737200, 4.740600, 4.744100, 4.747600, 4.751100, 4.754600, 4.758100, 4.761600, 4.765000, 4.768500, 4.772000,
    4.775500, 4.778900, 4.782400, 4.785900, 4.789300, 4.792800, 4.796300, 4.799700, 4.803200, 4.806700, 4.810100, 4.813600, 4.817000, 4.820500, 4.824000,
    4.827400, 4.830900, 4.834300, 4.837800, 4.841200, 4.844700, 4.848100, 4.851600, 4.855000, 4.858500, 4.861900, 4.865300, 4.868800, 4.872200, 4.875700,
    4.879100, 4.882500, 4.886000, 4.889400, 4.892900, 4.896300, 4.899700, 4.903200, 4.906600, 4.910000, 4.913500, 4.916900, 4.920300, 4.923700, 4.927200,
    4.930600, 4.934000, 4.937500, 4.940900, 4.944300, 4.947700, 4.951200, 4.954600, 4.958000, 4.961500, 4.964900, 4.968300, 4.971700, 4.975200, 4.978600,
    4.982000, 4.985400, 4.988900, 4.992300, 4.995700, 4.999100, 5.002500, 5.006000, 5.009400, 5.012800, 5.016200, 5.019700, 5.023100, 5.026500, 5.029900,
    5.033400, 5.036800, 5.040200, 5.043600, 5.047100, 5.050500, 5.053900, 5.057300, 5.060800, 5.064200, 5.067600, 5.071100, 5.074500, 5.077900, 5.081300,
    5.084800, 5.088200, 5.091600, 5.095100, 5.098500, 5.101900, 5.105400, 5.108800, 5.112200, 5.115700, 5.119100, 5.122500, 5.126000, 5.129400, 5.132900,
    5.136300, 5.139700, 5.143200, 5.146600, 5.150100, 5.153500, 5.157000, 5.160400, 5.163900, 5.167300, 5.170800, 5.174200, 5.177700, 5.181100, 5.184600,
    5.188000, 5.191500, 5.194900, 5.198400, 5.201900, 5.205300, 5.208800, 5.212200, 5.215700, 5.219200, 5.222600, 5.226100, 5.229600, 5.233100, 5.236500,
    5.240000, 5.243500, 5.247000, 5.250400, 5.253900, 5.257400, 5.260900, 5.264400, 5.267900, 5.271400, 5.274900, 5.278300, 5.281800, 5.285300, 5.288800,
    5.292300, 5.295800, 5.299300, 5.302900, 5.306400, 5.309900, 5.313400, 5.316900, 5.320400, 5.323900, 5.327500, 5.331000, 5.334500, 5.338000, 5.341600,
    5.345100, 5.348600, 5.352200, 5.355700, 5.359200, 5.362800, 5.366300, 5.369900, 5.373400, 5.377000, 5.380500, 5.384100, 5.387700, 5.391200, 5.394800,
    5.398400, 5.401900, 5.405500, 5.409100, 5.412700, 5.416200, 5.419800, 5.423400, 5.427000, 5.430600, 5.434200, 5.437800, 5.441400, 5.445000, 5.448600,
    5.452200, 5.455800, 5.459500, 5.463100, 5.466700, 5.470300, 5.474000, 5.477600, 5.481200, 5.484900, 5.488500, 5.492200, 5.495800, 5.499500, 5.503100,
    5.506800, 5.510500, 5.514100, 5.517800, 5.521500, 5.525100, 5.528800, 5.532500, 5.536200, 5.539900, 5.543600, 5.547300, 5.551000, 5.554700, 5.558400,
    5.562100, 5.565900, 5.569600, 5.573300, 5.577100, 5.580800, 5.584500, 5.588300, 5.592000, 5.595800, 5.599500, 5.603300, 5.607100, 5.610800, 5.614600,
    5.618400, 5.622200, 5.626000, 5.629800, 5.633600, 5.637400, 5.641200, 5.645000, 5.648800, 5.652600, 5.656500, 5.660300, 5.664100, 5.668000, 5.671800,
    5.675700, 5.679600, 5.683400, 5.687300, 5.691200, 5.695000, 5.698900, 5.702800, 5.706700, 5.710600, 5.714500, 5.718400, 5.722400, 5.726300, 5.730200,
    5.734100, 5.738100, 5.742000, 5.746000, 5.749900, 5.753900, 5.757900, 5.761900, 5.765800, 5.769800, 5.773800, 5.777800, 5.781800, 5.785800, 5.789900,
    5.793900, 5.797900, 5.802000, 5.806000, 5.810100, 5.814100, 5.818200, 5.822300, 5.826400, 5.830400, 5.834500, 5.838600, 5.842800, 5.846900, 5.851000,
    5.855100, 5.859300, 5.863400, 5.867600, 5.871700, 5.875900, 5.880100, 5.884300, 5.888500, 5.892700, 5.896900, 5.901100, 5.905300, 5.909500, 5.913800,
    5.918000, 5.922300, 5.926500, 5.930800, 5.935100, 5.939400, 5.943700, 5.948000, 5.952300, 5.956600, 5.961000, 5.965300, 5.969700, 5.974000, 5.978400,
    5.982800, 5.987200, 5.991600, 5.996000, 6.000400, 6.004800, 6.009300, 6.013700, 6.018200, 6.022700, 6.027100, 6.031600, 6.036100, 6.040600, 6.045200,
    6.049700, 6.054200, 6.058800, 6.063400, 6.067900, 6.072500, 6.077100, 6.081700, 6.086300, 6.091000, 6.095600, 6.100300, 6.104900, 6.109600, 6.114300,
    6.119000, 6.123700, 6.128500, 6.133200, 6.137900, 6.142700, 6.147500, 6.152300, 6.157100, 6.161900, 6.166700, 6.171600, 6.176400, 6.181300, 6.186200,
    6.191100, 6.196000, 6.200900, 6.205900, 6.210800, 6.215800, 6.220800, 6.225800, 6.230800, 6.235800, 6.240900, 6.245900, 6.251000, 6.256100, 6.261200,
    6.266300, 6.271500, 6.276600, 6.281800, 6.287000, 6.292200, 6.297400, 6.302700, 6.307900, 6.313200, 6.318500, 6.323800, 6.329100, 6.334500, 6.339900,
    6.345200, 6.350600, 6.356100, 6.361500, 6.367000, 6.372500, 6.378000, 6.383500, 6.389000, 6.394600, 6.400200, 6.405800, 6.411400, 6.417100, 6.422700,
    6.428400, 6.434100, 6.439900, 6.445600, 6.451400, 6.457200, 6.463100, 6.468900, 6.474800, 6.480700, 6.486600, 6.492600, 6.498600, 6.504600, 6.510600,
    6.516600, 6.522700, 6.528800, 6.535000, 6.541100, 6.547300, 6.553600, 6.559800, 6.566100, 6.572400, 6.578700, 6.585100, 6.591500, 6.597900, 6.604400,
    6.610900, 6.617400, 6.624000, 6.630600, 6.637200, 6.643800, 6.650500, 6.657300, 6.664000, 6.670800, 6.677700, 6.684500, 6.691500, 6.698400, 6.705400,
    6.712400, 6.719500, 6.726600, 6.733800, 6.740900, 6.748200, 6.755500, 6.762800, 6.770100, 6.777600, 6.785000, 6.792500, 6.800100, 6.807700, 6.815300,
    6.823000, 6.830700, 6.838500, 6.846400, 6.854300, 6.862300, 6.870300, 6.878300, 6.886500, 6.894700, 6.902900, 6.911200, 6.919600, 6.928000, 6.936500,
    6.945000, 6.953700, 6.962400, 6.971100, 6.979900, 6.988800, 6.997800, 7.006900, 7.016000, 7.025200, 7.034500, 7.043800, 7.053300, 7.062800, 7.072400,
    7.082100, 7.091900, 7.101800, 7.111800, 7.121800, 7.132000, 7.142300, 7.152700, 7.163200, 7.173800, 7.184500, 7.195300, 7.206200, 7.217300, 7.228500,
    7.239800, 7.251200, 7.262800, 7.274500, 7.286400, 7.298400, 7.310500, 7.322900, 7.335300, 7.348000, 7.360800, 7.373700, 7.386900, 7.400300, 7.413800,
    7.427600, 7.441500, 7.455700, 7.470100, 7.484700, 7.499500, 7.514700, 7.530000, 7.545700, 7.561600, 7.577800, 7.594300, 7.611200, 7.628300, 7.645800,
    7.663700, 7.682000, 7.700700, 7.719800, 7.739300, 7.759300, 7.779800, 7.800800, 7.822400, 7.844600, 7.867400, 7.890900, 7.915000, 7.940000, 7.965800,
    7.992400, 8.020000, 8.048700, 8.078400, 8.109400, 8.141800, 8.175600, 8.211100, 8.248400, 8.287700, 8.329400, 8.373700, 8.421000, 8.471900, 8.527000,
    8.587000, 8.653100, 8.726800, 8.810300, 8.907000, 9.022300, 9.166300, 9.360500, 9.669800, 10.858900
  };
  */
    const double Controller::SNR_PDR[SNR_PDR_SAMPLE_NUMBER][2] = { // with channel dynamics
      {-5, 0},
      {-4, 0},
      {-3, 0},
      {-2, 0},
      {-1, 0.001},
      {0, 0.004},
      {1, 0.021},
      {2, 0.071},
      {3, 0.191},
      {4, 0.329},
      {5, 0.452},
      {6, 0.63},
      {7, 0.767},
      {8, 0.83},
      {9, 0.883},
      {10, 0.949},
      {11, 0.968},
      {12, 0.984},
      {13, 0.987},
      {14, 0.997},
      {15, 1},
      {16, 1},
      {17, 1},
      {18, 1},
      {19, 1},
      {20, 1}
    };
  double Controller::ComputeSlope (double currentPdr)
  {
    double slope = 0.0; 
    if (currentPdr < 0.002)
    {
      currentPdr = 0.002;
    }

    for (int i = 0; i < SNR_PDR_SAMPLE_NUMBER; ++ i)
    {
      if (SNR_PDR[i][1] >= currentPdr)
      {
        slope = (SNR_PDR[i-1][1] - SNR_PDR[i][1] )/ (SNR_PDR[i-1][0] - SNR_PDR[i][0]);
        std::cout<<"lower_snr: "<< SNR_PDR[i-1][0]<<" higher_snr: "<< SNR_PDR[i][0]<<" higher_pdr: "<< SNR_PDR[i][1]<<" lower_pdr: "<< SNR_PDR[i-1][0]<< " slope: "<< slope << std::endl;
        break;
      }
    }

    /*
    uint32_t index = GetIndexByPdr (currentPdr);
    if (index == 0)
    {
      return 0;
    }
    double deltaSnr = SNR_PDR[index][0] - SNR_PDR[index -1][0];
    double deltaPdr = SNR_PDR[index][1] - SNR_PDR[index-1][1];
    slope = fabs (deltaPdr / deltaSnr);
    */
    return fabs (slope);
  }

  double Controller::ComputeSlope (double currentPdr, double desiredPdr)
  {
    NS_ASSERT (currentPdr != desiredPdr);
    double slope = 0.0; 
    if (currentPdr < 0.002)
    {
      currentPdr = 0.002;
    } 
    double lower_snr =0;
    double higher_snr = 0;
    for (int i = 0; i < SNR_PDR_SAMPLE_NUMBER; ++ i)
    {
      if ( SNR_PDR[i][1] >= currentPdr)
      {
        lower_snr = SNR_PDR[i-1][0];
      }
    }

    for (int i = 0; i < SNR_PDR_SAMPLE_NUMBER; ++ i)
    {
      if ( SNR_PDR[i][1] >= desiredPdr)
      {
        higher_snr= SNR_PDR[i][0];
      }
    }
    //double deltaSnr = SNR_PDR[GetIndexByPdr (currentPdr)][0] - SNR_PDR[ GetIndexByPdr (desiredPdr)][0];
    double deltaSnr = lower_snr - higher_snr;
    slope = fabs ((currentPdr - desiredPdr)/deltaSnr); // the slope at the point PDR=currentPdr in the SNR-to-PDR curve.
    std::cout<<"lower_snr: "<< lower_snr <<" higher_snr: "<< higher_snr <<" currentPdr: "<< currentPdr <<" desiredPdr: "<< desiredPdr << " slope: "<< slope << std::endl;
    return slope;
  }

  uint32_t Controller::GetIndexByPdr (double pdr)
  {
    if (pdr < 0.002)
    {
      return 0;
    }
    for (uint32_t i = 0; i < SNR_PDR_SAMPLE_NUMBER-1; ++ i)
    {
      if (SNR_PDR[i][1] >= pdr && SNR_PDR[i+1][1] < pdr)
      {
        return i;
      }
    }
    return 0;
  }

  double Controller::GetSnrByPdr (double pdr)
  {
    if (pdr < 0.002)
    {
      pdr = 0.002;
    }
    for (uint32_t i = 1; i < SNR_PDR_SAMPLE_NUMBER; ++ i)
    {
      if (SNR_PDR[i][1] > pdr && SNR_PDR[i-1][1] <= pdr)
      {
        return SNR_PDR[i][0];
      }
    }
    return  SNR_PDR[0][0];
  }


  NS_OBJECT_ENSURE_REGISTERED (PControllerWithReferenceInterference);

  TypeId PControllerWithReferenceInterference::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::PControllerWithReferenceInterference")
      .SetParent<Controller> ()
      .AddConstructor<PControllerWithReferenceInterference> ()
      ;
    return tid;
  }

  PControllerWithReferenceInterference::PControllerWithReferenceInterference ()
  {
    NS_LOG_FUNCTION (this);
  }

  PControllerWithReferenceInterference::~PControllerWithReferenceInterference ()
  {
    NS_LOG_FUNCTION (this);
  }

  double PControllerWithReferenceInterference::GetDeltaInterference(double rxPowerDbm, double expectedPdr, double currentNplusIDbm)
  {
#ifdef ENABLE_RID
    return 0;
#endif
    double a_0 = Controller::ComputeSlope (expectedPdr);
    double b_0 = expectedPdr - Controller::GetSnrByPdr (expectedPdr) * a_0;
    double I_r = rxPowerDbm + (b_0 - expectedPdr)/ a_0;
    double K_p = 1;
    double deltaInterferenceDb = K_p * (I_r - currentNplusIDbm);
    return deltaInterferenceDb;
  }




  NS_OBJECT_ENSURE_REGISTERED (PControllerWithDesiredPdr);

  TypeId PControllerWithDesiredPdr::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::PControllerWithDesiredPdr")
      .SetParent<Controller> ()
      .AddConstructor <PControllerWithDesiredPdr> ()
      ;
    return tid;
  }

  PControllerWithDesiredPdr::PControllerWithDesiredPdr ()
  {
    NS_LOG_FUNCTION (this);
  }

  PControllerWithDesiredPdr::~PControllerWithDesiredPdr ()
  {
    NS_LOG_FUNCTION (this);
  }


  double PControllerWithDesiredPdr::GetDeltaInterference(double desiredPdr, double currentPdr)
  {
#ifdef ENABLE_RID
    return 0;
#endif
    if (currentPdr == 0) 
    {
      currentPdr = 0.002; // the minimum pdr in the simulation;
    }
    double slope = 0.0;

#ifndef NO_PROTECTION // Use protection, E_0 is enabled.
    if (fabs (currentPdr - desiredPdr ) > E_0 )
    {
      slope = Controller::ComputeSlope (currentPdr, desiredPdr);
    }
    else if (fabs (currentPdr - desiredPdr ) <= E_0 )
    {
      slope = Controller::ComputeSlope (currentPdr);
    }
#endif

    double deltaSnr = 0.001/slope;
    double pParameter = -1.0/slope; 
    double deltaInterferenceDb = pParameter * (desiredPdr - currentPdr);
    std::cout<<"p_controller: slope: "<<slope<<"\tpParameter: "<<pParameter<<"\tdeltaSnr: "<<deltaSnr<<"\tdesiredPdr: "<< desiredPdr <<"\tcurrentPdr: "<< currentPdr<<"\tdeltaInterference: "<<deltaInterferenceDb<<std::endl;
    /*
       if (desiredPdr == currentPdr)
       {
       return 0;
       }
       */
    return deltaInterferenceDb;
  }

  NS_OBJECT_ENSURE_REGISTERED (MinimumVarianceController);
  TypeId MinimumVarianceController::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::MinimumVarianceController")
      .SetParent<Controller> ()
      .AddConstructor <MinimumVarianceController> ()
      ;
    return tid;
  }
  MinimumVarianceController::MinimumVarianceController ()
  {
    m_deltaY = DELTA_Y;
    m_ewmaCoefficient = 0.4;
    m_E0 = E_0;
    NS_LOG_FUNCTION (this);
  }
  MinimumVarianceController::~MinimumVarianceController ()
  {
    NS_LOG_FUNCTION (this);
  }
  double MinimumVarianceController::GetDeltaInterference (double desiredPdr, double ewmaCurrentPdr, double estimatedCurrentPdr, bool &conditionTwoMeet)
  {
#ifdef ENABLE_RID
    return 0;
#endif
    double slope = 0;
#ifndef NO_PROTECTION // Use protection, m_E0 is enabled.
    if (fabs (ewmaCurrentPdr - desiredPdr ) > m_E0 )
    {
      conditionTwoMeet = true;
      //slope = Controller::ComputeSlope (ewmaCurrentPdr) > Controller::ComputeSlope (desiredPdr) ? Controller::ComputeSlope (ewmaCurrentPdr) :Controller::ComputeSlope (desiredPdr);
      slope = Controller::ComputeSlope (ewmaCurrentPdr, desiredPdr);
    }
    else if (fabs (ewmaCurrentPdr - desiredPdr ) <= m_E0 )
    {
      slope = Controller::ComputeSlope (ewmaCurrentPdr);
    }
#endif
#ifdef NO_PROTECTION // Do not use protection, always use the current EWMA PDR.
    slope = Controller::ComputeSlope (ewmaCurrentPdr);
#endif
    double deltaInterferenceDb = (m_ewmaCoefficient * ewmaCurrentPdr + (1 - m_ewmaCoefficient) * estimatedCurrentPdr -  desiredPdr - m_deltaY ) /((1 - m_ewmaCoefficient) * slope);
    std::cout<<" controller: desired.pdr: "<<desiredPdr <<" ewma.current.pdr: "<< ewmaCurrentPdr <<" estimated.current.pdr: "<< estimatedCurrentPdr <<" delta.interference.db: "<< deltaInterferenceDb <<" 1/((1-m_ewmaCoefficient)*slope): "<< 1 / ((1- m_ewmaCoefficient) * slope )<<" slope: "<< slope << std::endl;
    return deltaInterferenceDb;
  }
}
