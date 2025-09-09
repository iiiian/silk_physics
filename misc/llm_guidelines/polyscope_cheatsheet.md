# Polyscope C++ API Cheatsheet

you can check api detail by examing header files located at build/debug/_deps/polyscope-src/include/polyscope.

## affine_remapper.h

```cpp
inline std::string defaultColorMap(DataType type);
```
Default color map.

```cpp
enum class VectorType { STANDARD = 0, AMBIENT };
```
What is the meaningful scale of an R3 vector? Used to scale vector lengths in a meaningful way STANDARD: no special meaning AMBIENT: vector represent distances in the ambient space

```cpp
template <class P> struct FIELD_MAG {
```
Struct FIELD_MAG.

```cpp
template <> struct FIELD_MAG<glm::vec3> {
```
Struct FIELD_MAG.

```cpp
template <typename T> std::pair<typename FIELD_MAG<T>::type, typename FIELD_MAG<T>::type> robustMinMax(const std::vector<T>& data, typename FIELD_MAG<T>::type rangeEPS = 1e-12);
```
Compute robust min max.

```cpp
template <typename T> class AffineRemapper {
```
Class AffineRemapper.

```cpp
AffineRemapper(const std::vector<T>& data, DataType datatype = DataType::STANDARD);
```
Affine remapper.

```cpp
AffineRemapper(T offset, typename FIELD_MAG<T>::type scale);
```
Affine remapper.

```cpp
AffineRemapper(typename FIELD_MAG<T>::type minVal, typename FIELD_MAG<T>::type maxVal, DataType datatype = DataType::STANDARD);
```
Affine remapper.

```cpp
AffineRemapper();
```
Affine remapper.

```cpp
T offset;
```
Offset parameter for the affine map.

```cpp
typename FIELD_MAG<T>::type scale, minVal, maxVal;
```
Scale factor for the affine map.

```cpp
typename FIELD_MAG<T>::type scale, minVal, maxVal;
```
Minimum value bound for mapping.

```cpp
typename FIELD_MAG<T>::type scale, minVal, maxVal;
```
Maximum value bound for mapping.

```cpp
T map(const T& x);
```
Apply the mapping to the input.

```cpp
void setMinMax(const std::vector<T>& data);
```
Set min max.

```cpp
std::string printBounds();
```
Print bounds.

```cpp
static T one();
```
One.

```cpp
static T zero();
```
Zero.

## camera_parameters.h

```cpp
class CameraIntrinsics {
```
Utility class to track the parameters of a camera Note that these DO NOT include any particular image discretization (which would be measured in pixels) Polyscope's cameras use openGL camera conventions.

```cpp
CameraIntrinsics();
```
Camera intrinsics.

```cpp
CameraIntrinsics(const float& FoVVerticalDegrees, const float& aspectRatioWidthOverHeight);
```
Camera intrinsics.

```cpp
static CameraIntrinsics fromFoVDegVerticalAndAspect(const float& fovVertDeg, const float& aspectRatioWidthOverHeight);
```
Construct from fo v deg vertical and aspect.

```cpp
static CameraIntrinsics fromFoVDegHorizontalAndAspect(const float& fovHorzDeg, const float& aspectRatioWidthOverHeight);
```
Construct from fo v deg horizontal and aspect.

```cpp
static CameraIntrinsics fromFoVDegHorizontalAndVertical(const float& fovHorzDeg, const float& fovVertDeg);
```
Construct from fo v deg horizontal and vertical.

```cpp
static CameraIntrinsics createInvalid();
```
Create invalid.

```cpp
bool isValid() const;
```
Return whether valid.

```cpp
float getFoVVerticalDegrees() const;
```
Get fo v vertical degrees.

```cpp
float getAspectRatioWidthOverHeight() const;
```
Get aspect ratio width over height.

```cpp
float fovVerticalDegrees;
```
Member fov vertical degrees.

```cpp
float aspectRatioWidthOverHeight;
```
Member aspect ratio width over height.

```cpp
bool isValidFlag;
```
Member is valid flag.

```cpp
class CameraExtrinsics {
```
Class CameraExtrinsics.

```cpp
CameraExtrinsics();
```
Camera extrinsics.

```cpp
CameraExtrinsics(const glm::mat4& E);
```
Camera extrinsics.

```cpp
template <class T1, class T2, class T3> static CameraExtrinsics fromVectors(const T1& root, const T2& lookDir, const T3& upDir);
```
Construct from vectors.

```cpp
static CameraExtrinsics fromMatrix(const glm::mat4& E);
```
Construct from matrix.

```cpp
static CameraExtrinsics createInvalid();
```
Create invalid.

```cpp
bool isValid() const;
```
Return whether valid.

```cpp
glm::vec3 getT() const;
```
Get t.

```cpp
glm::mat3x3 getR() const;
```
Get r.

```cpp
glm::mat4x4 getViewMat() const;
```
Get view mat.

```cpp
glm::mat4x4 getE() const;
```
Get e.

```cpp
glm::vec3 getPosition() const;
```
Get position.

```cpp
glm::vec3 getLookDir() const;
```
Get look dir.

```cpp
glm::vec3 getUpDir() const;
```
Get up dir.

```cpp
glm::vec3 getRightDir() const;
```
Get right dir.

```cpp
std::tuple<glm::vec3, glm::vec3, glm::vec3> getCameraFrame() const;
```
Get camera frame.

```cpp
glm::mat4x4 E;
```
Member e.

```cpp
bool isValidFlag;
```
Member is valid flag.

```cpp
class CameraParameters {
```
Class CameraParameters.

```cpp
CameraParameters();
```
Camera parameters.

```cpp
CameraParameters(const CameraIntrinsics& intrinsics, const CameraExtrinsics& extrinsics);
```
Camera parameters.

```cpp
static CameraParameters createInvalid();
```
Create invalid.

```cpp
bool isValid() const;
```
Return whether valid.

```cpp
bool isfinite() const;
```
Return whether finite.

```cpp
CameraIntrinsics intrinsics;
```
Member intrinsics.

```cpp
CameraExtrinsics extrinsics;
```
Member extrinsics.

```cpp
std::vector<glm::vec3> generateCameraRays(size_t dimX, size_t dimY, ImageOrigin origin = ImageOrigin::UpperLeft) const;
```
Generate camera rays.

```cpp
std::array<glm::vec3, 4> generateCameraRayCorners() const;
```
Generate camera ray corners.

```cpp
glm::vec3 getT() const;
```
Get t.

```cpp
glm::mat3x3 getR() const;
```
Get r.

```cpp
glm::mat4x4 getViewMat() const;
```
Get view mat.

```cpp
glm::mat4x4 getE() const;
```
Get e.

```cpp
glm::vec3 getPosition() const;
```
Get position.

```cpp
glm::vec3 getLookDir() const;
```
Get look dir.

```cpp
glm::vec3 getUpDir() const;
```
Get up dir.

```cpp
glm::vec3 getRightDir() const;
```
Get right dir.

```cpp
std::tuple<glm::vec3, glm::vec3, glm::vec3> getCameraFrame() const;
```
Get camera frame.

```cpp
float getFoVVerticalDegrees() const;
```
Get fo v vertical degrees.

```cpp
float getAspectRatioWidthOverHeight() const;
```
Get aspect ratio width over height.

## camera_view.h

```cpp
class CameraView;
```
Forward declare structure

```cpp
template <> // Specialize the quantity type struct QuantityTypeHelper<CameraView> {
```
Struct QuantityTypeHelper.

```cpp
*/ struct CameraViewPickResult {
```
Forward declare quantity types (currently there are none) template <> // Specialize the quantity type struct QuantityTypeHelper<CameraView> { typedef CameraViewQuantity type; };

```cpp
class CameraView : public QuantityStructure<CameraView> {
```
Class CameraView.

```cpp
CameraView(std::string name, const CameraParameters& params);
```
Camera view.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void buildPickUI(const PickResult& result) override;
```
Build pick ui.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
void updateCameraParameters(const CameraParameters& newParams);
```
Update camera parameters.

```cpp
CameraParameters getCameraParameters() const;
```
Get camera parameters.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
void deleteProgram();
```
Delete program.

```cpp
void setViewToThisCamera(bool withFlight = false);
```
Set view to this camera.

```cpp
CameraViewPickResult interpretPickResult(const PickResult& result);
```
Interpret pick result.

```cpp
CameraView* setWidgetFocalLength(float newVal, bool isRelative = true);
```
Set widget focal length.

```cpp
float getWidgetFocalLength();
```
Get widget focal length.

```cpp
CameraView* setWidgetThickness(float newVal);
```
Set widget thickness.

```cpp
float getWidgetThickness();
```
Get widget thickness.

```cpp
CameraView* setWidgetColor(glm::vec3 val);
```
Set widget color.

```cpp
glm::vec3 getWidgetColor();
```
Get widget color.

```cpp
void setCameraViewUniforms(render::ShaderProgram& p);
```
Set camera view uniforms.

```cpp
std::vector<std::string> addCameraViewRules(std::vector<std::string> initRules, bool withCameraView = true);
```
Add camera view rules.

```cpp
std::string getShaderNameForRenderMode();
```
Get shader name for render mode.

```cpp
std::tuple<glm::vec3, glm::vec3, glm::vec3> getFrameBillboardGeometry();
```
Get frame billboard geometry.

```cpp
CameraParameters params;
```
Member params.

```cpp
PersistentValue<ScaledValue<float>> widgetFocalLength;
```
Member widget focal length.

```cpp
PersistentValue<float> widgetThickness;
```
Member widget thickness.

```cpp
PersistentValue<glm::vec3> widgetColor;
```
Member widget color.

```cpp
std::shared_ptr<render::ShaderProgram> nodeProgram, edgeProgram;
```
Member node program.

```cpp
std::shared_ptr<render::ShaderProgram> nodeProgram, edgeProgram;
```
Member edge program.

```cpp
std::shared_ptr<render::ShaderProgram> pickFrameProgram;
```
Member pick frame program.

```cpp
void prepare();
```
Prepare.

```cpp
void preparePick();
```
Prepare pick.

```cpp
void geometryChanged();
```
Geometry changed.

```cpp
void fillCameraWidgetGeometry(render::ShaderProgram* nodeProgram, render::ShaderProgram* edgeProgram, render::ShaderProgram* pickFrameProgram);
```
Fill camera widget geometry.

```cpp
size_t pickStart = INVALID_IND;
```
Member invalid ind.

```cpp
glm::vec3 pickColor;
```
Member pick color.

```cpp
CameraView* registerCameraView(std::string name, CameraParameters params);
```
Register camera view.

```cpp
inline CameraView* getCameraView(std::string name = "");
```
Get camera view.

```cpp
inline bool hasCameraView(std::string name = "");
```
Return whether it has camera view.

```cpp
inline void removeCameraView(std::string name = "", bool errorIfAbsent = false);
```
Remove camera view.

## check_invalid_values.h

```cpp
template <typename T> void checkInvalidValues(std::string name, const std::vector<T>& data) { if (options::warnForInvalidValues) { for (const T& val : data) { if (!allComponentsFinite(val)) { info("Invalid +-inf or NaN values detected in buffer: " + name);
```
Check invalid values.

## color_image_quantity.h

```cpp
class ColorImageQuantity : public ImageQuantity {
```
Class ColorImageQuantity.

```cpp
ColorImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<glm::vec4>& data, ImageOrigin imageOrigin);
```
Color image quantity.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
render::ManagedBuffer<glm::vec4> colors;
```
Member colors.

```cpp
ColorImageQuantity* setEnabled(bool newEnabled) override;
```
Set enabled.

```cpp
ColorImageQuantity* setIsPremultiplied(bool val);
```
Set is premultiplied.

```cpp
bool getIsPremultiplied();
```
Get is premultiplied.

```cpp
std::vector<glm::vec4> colorsData;
```
Member colors data.

```cpp
PersistentValue<bool> isPremultiplied;
```
Member is premultiplied.

```cpp
std::shared_ptr<render::ShaderProgram> fullscreenProgram, billboardProgram;
```
Member fullscreen program.

```cpp
std::shared_ptr<render::ShaderProgram> fullscreenProgram, billboardProgram;
```
Member billboard program.

```cpp
void prepareFullscreen();
```
Prepare fullscreen.

```cpp
void prepareBillboard();
```
Prepare billboard.

```cpp
virtual void showFullscreen() override;
```
Show fullscreen.

```cpp
virtual void showInImGuiWindow() override;
```
Show in im gui window.

```cpp
virtual void showInBillboard(glm::vec3 center, glm::vec3 upVec, glm::vec3 rightVec) override;
```
Show in billboard.

## color_management.h

```cpp
glm::vec3 RGBtoHSV(glm::vec3 rgb);
```
Rg bto hsv.

```cpp
glm::vec3 HSVtoRGB(glm::vec3 hsv);
```
Hs vto rgb.

```cpp
glm::vec3 getNextUniqueColor();
```
Get next unique color.

## color_quantity.h

```cpp
template <typename QuantityT> class ColorQuantity {
```
Class ColorQuantity.

```cpp
ColorQuantity(QuantityT& parent, const std::vector<glm::vec3>& colors);
```
Color quantity.

```cpp
void buildColorUI();
```
Build color ui.

```cpp
virtual void buildColorOptionsUI();
```
Build color options ui.

```cpp
std::vector<std::string> addColorRules(std::vector<std::string> rules);
```
Add color rules.

```cpp
void setColorUniforms(render::ShaderProgram& p);
```
Set color uniforms.

```cpp
template <class V> void updateData(const V& newColors);
```
Update data.

```cpp
QuantityT& quantity;
```
Member quantity.

```cpp
render::ManagedBuffer<glm::vec3> colors;
```
Member colors.

```cpp
std::vector<glm::vec3> colorsData;
```
Member colors data.

## color_render_image_quantity.h

```cpp
class ColorRenderImageQuantity : public RenderImageQuantityBase {
```
Class ColorRenderImageQuantity.

```cpp
ColorRenderImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& normalData, const std::vector<glm::vec3>& colorsData, ImageOrigin imageOrigin);
```
Color render image quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
render::ManagedBuffer<glm::vec3> colors;
```
Member colors.

```cpp
template <typename T1, typename T2, typename T3> void updateBuffers(const T1& depthData, const T2& normalData, const T3& colorsData);
```
Update buffers.

```cpp
std::vector<glm::vec3> colorsData;
```
Member colors data.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
void prepare();
```
Prepare.

```cpp
template <typename T1, typename T2, typename T3> void ColorRenderImageQuantity::updateBuffers(const T1& depthData, const T2& normalData, const T3& colorsData) { validateSize(depthData, dimX * dimY, "color render image depth data " + name);
```
Update buffers.

```cpp
validateSize(normalData, {dimX * dimY, 0}, "color render image normal data " + name);
```
Validate size.

```cpp
validateSize(colorsData, dimX * dimY, "color render image color data " + name);
```
Validate size.

```cpp
std::vector<float> standardDepth(standardizeArray<float>(depthData));
```
Standard depth.

```cpp
std::vector<glm::vec3> standardNormal(standardizeVectorArray<glm::vec3, 3>(normalData));
```
Standard normal.

```cpp
std::vector<glm::vec3> standardColor(standardizeVectorArray<glm::vec3, 3>(colorsData));
```
Standard color.

```cpp
colors.markHostBufferUpdated();
```
Mark host buffer updated.

```cpp
updateBaseBuffers(standardDepth, standardNormal);
```
Update base buffers.

## colors.h

_No API symbols detected._

## combining_hash_functions.h

```cpp
template <typename TT> struct hash {
```
Struct hash.

```cpp
size_t operator()(TT const& tt) const { return std::hash<TT>()(tt); }
```
Operator.

```cpp
template <class T> inline void hash_combine(std::size_t& seed, T const& v) { seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
```
Return whether it has h combine.

```cpp
template <class Tuple, size_t Index = std::tuple_size<Tuple>::value - 1> struct HashValueImpl {
```
Struct HashValueImpl.

```cpp
static void apply(size_t& seed, Tuple const& tuple) { HashValueImpl<Tuple, Index - 1>::apply(seed, tuple);
```
Apply the mapping to the input.

```cpp
hash_combine(seed, std::get<Index>(tuple));
```
Return whether it has h combine.

```cpp
template <class Tuple> struct HashValueImpl<Tuple, 0> {
```
Struct HashValueImpl.

```cpp
static void apply(size_t& seed, Tuple const& tuple) { hash_combine(seed, std::get<0>(tuple)); }
```
Apply the mapping to the input.

```cpp
} // namespace template <typename... TT> struct hash<std::tuple<TT...>> {
```
Struct hash.

```cpp
size_t operator()(std::tuple<TT...> const& tt) const { size_t seed = 0;
```
Operator.

```cpp
HashValueImpl<std::tuple<TT...>>::apply(seed, tt);
```
Apply the mapping to the input.

```cpp
return seed;
```
Member seed.

```cpp
template <typename T, typename U> struct hash<std::pair<T, U>> {
```
Struct hash.

```cpp
std::size_t operator()(const std::pair<T, U>& x) const { size_t hVal = std::hash<T>()(x.first);
```
Operator.

```cpp
hash_combine<U>(hVal, x.second);
```
Function.

```cpp
return hVal;
```
Member h val.

```cpp
template <class T, size_t N> struct hash<std::array<T, N>> {
```
Struct hash.

```cpp
std::size_t operator()(const std::array<T, N>& arr) const { std::hash<T> hasher;
```
Operator.

```cpp
for (size_t i = 0; i < N; i++) {
```
For.

```cpp
hash_combine(result, arr[i]);
```
Return whether it has h combine.

```cpp
} return result;
```
Member result.

## context.h

```cpp
class Structure;
```
forward declarations

```cpp
class Group;
```
Class Group.

```cpp
class SlicePlane;
```
Class SlicePlane.

```cpp
class Widget;
```
Class Widget.

```cpp
class FloatingQuantityStructure;
```
Class FloatingQuantityStructure.

```cpp
} // namespace view struct Context {
```
A context object wrapping all global state used by Polyscope.

```cpp
bool initialized = false;
```
Member false.

```cpp
std::map<std::string, std::map<std::string, std::unique_ptr<Structure>>> structures;
```
Member string.

```cpp
std::map<std::string, std::map<std::string, std::unique_ptr<Structure>>> structures;
```
Member string.

```cpp
std::map<std::string, std::map<std::string, std::unique_ptr<Structure>>> structures;
```
Member structures.

```cpp
std::map<std::string, std::unique_ptr<Group>> groups;
```
Member string.

```cpp
std::map<std::string, std::unique_ptr<Group>> groups;
```
Member groups.

```cpp
std::tuple<glm::vec3, glm::vec3> boundingBox = std::tuple<glm::vec3, glm::vec3>{glm::vec3{-1., -1., -1.}, glm::vec3{1., 1., 1.}};
```
Member vec 3.

```cpp
std::tuple<glm::vec3, glm::vec3> boundingBox = std::tuple<glm::vec3, glm::vec3>{glm::vec3{-1., -1., -1.}, glm::vec3{1., 1., 1.}};
```
Member vec 3.

```cpp
std::vector<std::unique_ptr<SlicePlane>> slicePlanes;
```
Member slice planes.

```cpp
std::vector<WeakHandle<Widget>> widgets;
```
Member widgets.

```cpp
bool doDefaultMouseInteraction = true;
```
Member true.

```cpp
std::function<void()> userCallback = nullptr;
```
Void.

```cpp
bool windowResizable = true;
```
Member true.

```cpp
NavigateStyle navigateStyle = NavigateStyle::Turntable;
```
Member turntable.

```cpp
UpDir upDir = UpDir::YUp;
```
Member y up.

```cpp
FrontDir frontDir = FrontDir::ZFront;
```
Member z front.

```cpp
double nearClipRatio = view::defaultNearClipRatio;
```
Member default near clip ratio.

```cpp
double farClipRatio = view::defaultFarClipRatio;
```
Member default far clip ratio.

```cpp
std::array<float, 4> bgColor{{1.0, 1.0, 1.0, 0.0}};
```
Member float.

```cpp
glm::mat4x4 viewMat;
```
Member view mat.

```cpp
double fov = view::defaultFov;
```
Member default fov.

```cpp
ProjectionMode projectionMode = ProjectionMode::Perspective;
```
Member perspective.

```cpp
glm::vec3 viewCenter;
```
Member view center.

```cpp
bool midflight = false;
```
Member false.

```cpp
glm::dualquat flightTargetViewR, flightInitialViewR;
```
Member flight target view r.

```cpp
glm::dualquat flightTargetViewR, flightInitialViewR;
```
Member flight initial view r.

```cpp
glm::vec3 flightTargetViewT, flightInitialViewT;
```
Member flight target view t.

```cpp
glm::vec3 flightTargetViewT, flightInitialViewT;
```
Member flight initial view t.

```cpp
float flightTargetFov, flightInitialFov;
```
Member flight target fov.

```cpp
float flightTargetFov, flightInitialFov;
```
Member flight initial fov.

```cpp
PickResult currSelectionPickResult;
```
Member curr selection pick result.

```cpp
bool haveSelectionVal = false;
```
Member false.

```cpp
std::unordered_map<Structure*, std::tuple<uint64_t, uint64_t>> structureRanges;
```
Member uint 64 t.

```cpp
std::unordered_map<Structure*, std::tuple<uint64_t, uint64_t>> structureRanges;
```
Member structure ranges.

```cpp
bool pointCloudEfficiencyWarningReported = false;
```
Member false.

```cpp
FloatingQuantityStructure* globalFloatingQuantityStructure = nullptr;
```
Member nullptr.

## curve_network.h

```cpp
class CurveNetwork;
```
Forward declare curve network

```cpp
class CurveNetworkNodeScalarQuantity;
```
Forward declare quantity types

```cpp
class CurveNetworkEdgeScalarQuantity;
```
Class CurveNetworkEdgeScalarQuantity.

```cpp
class CurveNetworkNodeColorQuantity;
```
Class CurveNetworkNodeColorQuantity.

```cpp
class CurveNetworkEdgeColorQuantity;
```
Class CurveNetworkEdgeColorQuantity.

```cpp
class CurveNetworkNodeVectorQuantity;
```
Class CurveNetworkNodeVectorQuantity.

```cpp
class CurveNetworkEdgeVectorQuantity;
```
Class CurveNetworkEdgeVectorQuantity.

```cpp
template <> // Specialize the quantity type struct QuantityTypeHelper<CurveNetwork> {
```
Struct QuantityTypeHelper.

```cpp
struct CurveNetworkPickResult {
```
Struct CurveNetworkPickResult.

```cpp
CurveNetworkElement elementType;
```
Member element type.

```cpp
int64_t index;
```
Member index.

```cpp
class CurveNetwork : public QuantityStructure<CurveNetwork> {
```
Class CurveNetwork.

```cpp
CurveNetwork(std::string name, std::vector<glm::vec3> nodes, std::vector<std::array<size_t, 2>> edges);
```
Curve network.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void buildPickUI(const PickResult&) override;
```
Build pick ui.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
render::ManagedBuffer<glm::vec3> nodePositions;
```
Member node positions.

```cpp
render::ManagedBuffer<uint32_t> edgeTailInds;
```
Member edge tail inds.

```cpp
render::ManagedBuffer<uint32_t> edgeTipInds;
```
Member edge tip inds.

```cpp
render::ManagedBuffer<glm::vec3> edgeCenters;
```
Member edge centers.

```cpp
template <class T> CurveNetworkNodeScalarQuantity* addNodeScalarQuantity(std::string name, const T& values, DataType type = DataType::STANDARD);
```
Add node scalar quantity.

```cpp
template <class T> CurveNetworkEdgeScalarQuantity* addEdgeScalarQuantity(std::string name, const T& values, DataType type = DataType::STANDARD);
```
Add edge scalar quantity.

```cpp
template <class T> CurveNetworkNodeColorQuantity* addNodeColorQuantity(std::string name, const T& values);
```
Add node color quantity.

```cpp
template <class T> CurveNetworkEdgeColorQuantity* addEdgeColorQuantity(std::string name, const T& values);
```
Add edge color quantity.

```cpp
template <class T> CurveNetworkNodeVectorQuantity* addNodeVectorQuantity(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add node vector quantity.

```cpp
template <class T> CurveNetworkEdgeVectorQuantity* addEdgeVectorQuantity(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add edge vector quantity.

```cpp
template <class T> CurveNetworkNodeVectorQuantity* addNodeVectorQuantity2D(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add node vector quantity 2 d.

```cpp
template <class T> CurveNetworkEdgeVectorQuantity* addEdgeVectorQuantity2D(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add edge vector quantity 2 d.

```cpp
std::vector<size_t> nodeDegrees;
```
Member node degrees.

```cpp
size_t nNodes() { return nodePositions.size(); }
```
N nodes.

```cpp
size_t nEdges() { return edgeTailInds.size(); }
```
N edges.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
void setCurveNetworkNodeUniforms(render::ShaderProgram& p);
```
Set curve network node uniforms.

```cpp
void setCurveNetworkEdgeUniforms(render::ShaderProgram& p);
```
Set curve network edge uniforms.

```cpp
void fillEdgeGeometryBuffers(render::ShaderProgram& program);
```
Fill edge geometry buffers.

```cpp
void fillNodeGeometryBuffers(render::ShaderProgram& program);
```
Fill node geometry buffers.

```cpp
std::vector<std::string> addCurveNetworkNodeRules(std::vector<std::string> initRules);
```
Add curve network node rules.

```cpp
std::vector<std::string> addCurveNetworkEdgeRules(std::vector<std::string> initRules);
```
Add curve network edge rules.

```cpp
template <class V> void updateNodePositions(const V& newPositions);
```
Update node positions.

```cpp
template <class V> void updateNodePositions2D(const V& newPositions);
```
Update node positions 2 d.

```cpp
CurveNetworkPickResult interpretPickResult(const PickResult& result);
```
Interpret pick result.

```cpp
CurveNetwork* setColor(glm::vec3 newVal);
```
Set color.

```cpp
glm::vec3 getColor();
```
Get color.

```cpp
void setNodeRadiusQuantity(CurveNetworkNodeScalarQuantity* quantity, bool autoScale = true);
```
Set node radius quantity.

```cpp
void setNodeRadiusQuantity(std::string name, bool autoScale = true);
```
Set node radius quantity.

```cpp
void clearNodeRadiusQuantity();
```
Clear node radius quantity.

```cpp
void setEdgeRadiusQuantity(CurveNetworkEdgeScalarQuantity* quantity, bool autoScale = true);
```
Set edge radius quantity.

```cpp
void setEdgeRadiusQuantity(std::string name, bool autoScale = true);
```
Set edge radius quantity.

```cpp
void clearEdgeRadiusQuantity();
```
Clear edge radius quantity.

```cpp
CurveNetwork* setRadius(float newVal, bool isRelative = true);
```
Set radius.

```cpp
float getRadius();
```
Get radius.

```cpp
CurveNetwork* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
std::vector<glm::vec3> nodePositionsData;
```
Member node positions data.

```cpp
std::vector<uint32_t> edgeTailIndsData;
```
Member edge tail inds data.

```cpp
std::vector<uint32_t> edgeTipIndsData;
```
Member edge tip inds data.

```cpp
std::vector<glm::vec3> edgeCentersData;
```
Member edge centers data.

```cpp
void computeEdgeCenters();
```
Compute edge centers.

```cpp
PersistentValue<glm::vec3> color;
```
Member color.

```cpp
PersistentValue<ScaledValue<float>> radius;
```
Member radius.

```cpp
PersistentValue<std::string> material;
```
Member material.

```cpp
std::shared_ptr<render::ShaderProgram> edgeProgram;
```
Member edge program.

```cpp
std::shared_ptr<render::ShaderProgram> nodeProgram;
```
Member node program.

```cpp
std::shared_ptr<render::ShaderProgram> edgePickProgram;
```
Member edge pick program.

```cpp
std::shared_ptr<render::ShaderProgram> nodePickProgram;
```
Member node pick program.

```cpp
void prepare();
```
Prepare.

```cpp
void preparePick();
```
Prepare pick.

```cpp
void recomputeGeometryIfPopulated();
```
Recompute geometry if populated.

```cpp
float computeNodeRadiusMultiplierUniform();
```
Compute node radius multiplier uniform.

```cpp
float computeEdgeRadiusMultiplierUniform();
```
Compute edge radius multiplier uniform.

```cpp
void buildNodePickUI(const CurveNetworkPickResult& result);
```
Build node pick ui.

```cpp
void buildEdgePickUI(const CurveNetworkPickResult& result);
```
Build edge pick ui.

```cpp
CurveNetworkNodeScalarQuantity* addNodeScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add node scalar quantity impl.

```cpp
CurveNetworkEdgeScalarQuantity* addEdgeScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add edge scalar quantity impl.

```cpp
CurveNetworkNodeColorQuantity* addNodeColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
```
Add node color quantity impl.

```cpp
CurveNetworkEdgeColorQuantity* addEdgeColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
```
Add edge color quantity impl.

```cpp
CurveNetworkNodeVectorQuantity* addNodeVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors, VectorType vectorType);
```
Add node vector quantity impl.

```cpp
CurveNetworkEdgeVectorQuantity* addEdgeVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors, VectorType vectorType);
```
Add edge vector quantity impl.

```cpp
bool nodeRadiusQuantityAutoscale = true;
```
Member true.

```cpp
bool edgeRadiusQuantityAutoscale = true;
```
Member true.

```cpp
CurveNetworkNodeScalarQuantity& resolveNodeRadiusQuantity();
```
Resolve node radius quantity.

```cpp
CurveNetworkEdgeScalarQuantity& resolveEdgeRadiusQuantity();
```
Resolve edge radius quantity.

```cpp
template <class P, class E> CurveNetwork* registerCurveNetwork(std::string name, const P& points, const E& edges);
```
Register curve network.

```cpp
template <class P, class E> template <class P, class E> CurveNetwork* registerCurveNetwork2D(std::string name, const P& points, const E& edges);
```
Register curve network 2 d.

```cpp
template <class P, class E> template <class P> CurveNetwork* registerCurveNetworkLine(std::string name, const P& points);
```
Register curve network line.

```cpp
template <class P> template <class P> CurveNetwork* registerCurveNetworkLine2D(std::string name, const P& points);
```
Register curve network line 2 d.

```cpp
template <class P> template <class P> CurveNetwork* registerCurveNetworkSegments(std::string name, const P& points);
```
Register curve network segments.

```cpp
template <class P> template <class P> CurveNetwork* registerCurveNetworkSegments2D(std::string name, const P& points);
```
Register curve network segments 2 d.

```cpp
template <class P> template <class P> CurveNetwork* registerCurveNetworkLoop(std::string name, const P& points);
```
Register curve network loop.

```cpp
template <class P> template <class P> CurveNetwork* registerCurveNetworkLoop2D(std::string name, const P& points);
```
Register curve network loop 2 d.

```cpp
inline CurveNetwork* getCurveNetwork(std::string name = "");
```
Get curve network.

```cpp
inline bool hasCurveNetwork(std::string name = "");
```
Return whether it has curve network.

```cpp
inline void removeCurveNetwork(std::string name = "", bool errorIfAbsent = false);
```
Remove curve network.

## curve_network_color_quantity.h

```cpp
class CurveNetworkMeshQuantity;
```
forward declaration

```cpp
class CurveNetwork;
```
Class CurveNetwork.

```cpp
class CurveNetworkColorQuantity : public CurveNetworkQuantity, public ColorQuantity<CurveNetworkColorQuantity> {
```
Class CurveNetworkColorQuantity.

```cpp
CurveNetworkColorQuantity(std::string name, CurveNetwork& network_, std::string definedOn, const std::vector<glm::vec3>& colorValues);
```
Curve network color quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
const std::string definedOn;
```
Member defined on.

```cpp
std::shared_ptr<render::ShaderProgram> nodeProgram;
```
Member node program.

```cpp
std::shared_ptr<render::ShaderProgram> edgeProgram;
```
Member edge program.

```cpp
virtual void createProgram() = 0;
```
Create program.

```cpp
class CurveNetworkNodeColorQuantity : public CurveNetworkColorQuantity {
```
======================================================== ========== Node Color ========== ========================================================

```cpp
CurveNetworkNodeColorQuantity(std::string name, std::vector<glm::vec3> values_, CurveNetwork& network_);
```
Curve network node color quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
void buildNodeInfoGUI(size_t vInd) override;
```
Build node info gui.

```cpp
class CurveNetworkEdgeColorQuantity : public CurveNetworkColorQuantity {
```
======================================================== ========== Edge Color ========== ========================================================

```cpp
CurveNetworkEdgeColorQuantity(std::string name, std::vector<glm::vec3> values_, CurveNetwork& network_);
```
Curve network edge color quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
void buildEdgeInfoGUI(size_t eInd) override;
```
Build edge info gui.

```cpp
render::ManagedBuffer<glm::vec3> nodeAverageColors;
```
Member node average colors.

```cpp
void updateNodeAverageColors();
```
Update node average colors.

```cpp
std::vector<glm::vec3> nodeAverageColorsData;
```
Member node average colors data.

## curve_network_quantity.h

```cpp
class CurveNetwork;
```
Forward declare

```cpp
class CurveNetworkQuantity : public QuantityS<CurveNetwork> {
```
Extend Quantity<CurveNetwork>

```cpp
CurveNetworkQuantity(std::string name, CurveNetwork& parentStructure, bool dominates = false);
```
Curve network quantity.

```cpp
virtual ~CurveNetworkQuantity() {};
```
Curve network quantity.

```cpp
virtual void buildNodeInfoGUI(size_t vInd);
```
Build node info gui.

```cpp
virtual void buildEdgeInfoGUI(size_t fInd);
```
Build edge info gui.

## curve_network_scalar_quantity.h

```cpp
class CurveNetworkScalarQuantity : public CurveNetworkQuantity, public ScalarQuantity<CurveNetworkScalarQuantity> {
```
Class CurveNetworkScalarQuantity.

```cpp
CurveNetworkScalarQuantity(std::string name, CurveNetwork& network_, std::string definedOn, const std::vector<float>& values, DataType dataType);
```
Curve network scalar quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
const std::string definedOn;
```
Member defined on.

```cpp
std::shared_ptr<render::ShaderProgram> nodeProgram;
```
Member node program.

```cpp
std::shared_ptr<render::ShaderProgram> edgeProgram;
```
Member edge program.

```cpp
virtual void createProgram() = 0;
```
Create program.

```cpp
class CurveNetworkNodeScalarQuantity : public CurveNetworkScalarQuantity {
```
======================================================== ========== Node Scalar ========== ========================================================

```cpp
CurveNetworkNodeScalarQuantity(std::string name, const std::vector<float>& values_, CurveNetwork& network_, DataType dataType_ = DataType::STANDARD);
```
Curve network node scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
void buildNodeInfoGUI(size_t nInd) override;
```
Build node info gui.

```cpp
class CurveNetworkEdgeScalarQuantity : public CurveNetworkScalarQuantity {
```
======================================================== ========== Edge Scalar ========== ========================================================

```cpp
CurveNetworkEdgeScalarQuantity(std::string name, const std::vector<float>& values_, CurveNetwork& network_, DataType dataType_ = DataType::STANDARD);
```
Curve network edge scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
void buildEdgeInfoGUI(size_t edgeInd) override;
```
Build edge info gui.

```cpp
render::ManagedBuffer<float> nodeAverageValues;
```
Member node average values.

```cpp
void updateNodeAverageValues();
```
Update node average values.

```cpp
std::vector<float> nodeAverageValuesData;
```
Member node average values data.

## curve_network_vector_quantity.h

```cpp
class CurveNetworkVectorQuantity : public CurveNetworkQuantity {
```
==== Common base class Represents a vector field associated with a curve network NOTE: This intermediate class is not really necessary anymore; it is subsumed by the VectorQuantity<> classes which serve as common bases for ALL vector types.

```cpp
CurveNetworkVectorQuantity(std::string name, CurveNetwork& network_);
```
Curve network vector quantity.

```cpp
class CurveNetworkNodeVectorQuantity : public CurveNetworkVectorQuantity, class CurveNetworkNodeVectorQuantity : public CurveNetworkVectorQuantity, public VectorQuantity<CurveNetworkNodeVectorQuantity> {
```
Class CurveNetworkNodeVectorQuantity.

```cpp
CurveNetworkNodeVectorQuantity(std::string name, std::vector<glm::vec3> vectors_, CurveNetwork& network_, VectorType vectorType_ = VectorType::STANDARD);
```
Curve network node vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual void buildNodeInfoGUI(size_t vInd) override;
```
Build node info gui.

```cpp
class CurveNetworkEdgeVectorQuantity : public CurveNetworkVectorQuantity, class CurveNetworkEdgeVectorQuantity : public CurveNetworkVectorQuantity, public VectorQuantity<CurveNetworkEdgeVectorQuantity> {
```
Class CurveNetworkEdgeVectorQuantity.

```cpp
CurveNetworkEdgeVectorQuantity(std::string name, std::vector<glm::vec3> vectors_, CurveNetwork& network_, VectorType vectorType_ = VectorType::STANDARD);
```
Curve network edge vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual void buildEdgeInfoGUI(size_t vInd) override;
```
Build edge info gui.

## depth_render_image_quantity.h

```cpp
class DepthRenderImageQuantity : public RenderImageQuantityBase {
```
Class DepthRenderImageQuantity.

```cpp
DepthRenderImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& normalData, ImageOrigin imageOrigin);
```
Depth render image quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
template <typename T1, typename T2> void updateBuffers(const T1& depthData, const T2& normalData);
```
Update buffers.

```cpp
DepthRenderImageQuantity* setColor(glm::vec3 newVal);
```
Set color.

```cpp
glm::vec3 getColor();
```
Get color.

```cpp
PersistentValue<glm::vec3> color;
```
Member color.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
void prepare();
```
Prepare.

```cpp
template <typename T1, typename T2> void DepthRenderImageQuantity::updateBuffers(const T1& depthData, const T2& normalData) { validateSize(depthData, dimX * dimY, "depth render image depth data " + name);
```
Update buffers.

```cpp
validateSize(normalData, {dimX * dimY, 0}, "depth render image normal data " + name);
```
Validate size.

```cpp
std::vector<float> standardDepth(standardizeArray<float>(depthData));
```
Standard depth.

```cpp
std::vector<glm::vec3> standardNormal(standardizeVectorArray<glm::vec3, 3>(normalData));
```
Standard normal.

```cpp
updateBaseBuffers(standardDepth, standardNormal);
```
Update base buffers.

## disjoint_sets.h

```cpp
class DisjointSets {
```
Class DisjointSets.

```cpp
DisjointSets(size_t n_);
```
Disjoint sets.

```cpp
size_t find(size_t x);
```
Find.

```cpp
void merge(size_t x, size_t y);
```
Merge.

```cpp
size_t n;
```
Member n.

```cpp
std::vector<size_t> parent;
```
Member parent.

```cpp
std::vector<size_t> rank;
```
Member rank.

```cpp
class MarkedDisjointSets {
```
Slight generalization of a disjoint set, which can track "marked" sets.

```cpp
MarkedDisjointSets(size_t n_);
```
Marked disjoint sets.

```cpp
size_t find(size_t x);
```
Find.

```cpp
void merge(size_t x, size_t y);
```
Merge.

```cpp
void mark(size_t x);
```
Mark.

```cpp
void unmark(size_t x);
```
Unmark.

```cpp
bool isMarked(size_t x);
```
Return whether marked.

```cpp
size_t n;
```
Member n.

```cpp
std::vector<size_t> parent;
```
Member parent.

```cpp
std::vector<size_t> rank;
```
Member rank.

```cpp
std::vector<bool> marked;
```
Member marked.

## elementary_geometry.h

```cpp
float computeTValAlongLine(glm::vec3 queryP, glm::vec3 lineStart, glm::vec3 lineEnd);
```
Compute t val along line.

```cpp
glm::vec3 projectToPlane(glm::vec3 queryP, glm::vec3 planeNormal, glm::vec3 pointOnPlane);
```
Project to plane.

```cpp
float signedTriangleArea(glm::vec3 normal, glm::vec3 pA, glm::vec3 pB, glm::vec3 pC);
```
Signed triangle area.

## file_helpers.h

```cpp
std::string promptForFilename(std::string filename = "out");
```
Prompt for filename.

## floating_quantities.h

_No API symbols detected._

## floating_quantity.h

```cpp
class FloatingQuantity : public Quantity {
```
Extend Quantity<> to add a few extra functions

```cpp
FloatingQuantity(std::string name, Structure& parentStructure);
```
Floating quantity.

```cpp
virtual ~FloatingQuantity() {};
```
Floating quantity.

```cpp
virtual void buildUI() override;
```
Build ui.

```cpp
virtual FloatingQuantity* setEnabled(bool newEnabled) = 0;
```
Set enabled.

## floating_quantity_structure.h

```cpp
class FloatingQuantityStructure;
```
Forward declare the structure

```cpp
class Quantity;
```
Forward declare quantity types

```cpp
class ScalarImageQuantity;
```
Class ScalarImageQuantity.

```cpp
class ColorImageQuantity;
```
Class ColorImageQuantity.

```cpp
class FloatingQuantityStructure : public QuantityStructure<FloatingQuantityStructure> {
```
Class FloatingQuantityStructure.

```cpp
FloatingQuantityStructure(std::string name);
```
Floating quantity structure.

```cpp
~FloatingQuantityStructure();
```
Floating quantity structure.

```cpp
virtual void buildUI() override;
```
Build ui.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildPickUI(const PickResult& result) override;
```
Build pick ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual bool hasExtents() override;
```
Return whether it has extents.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
FloatingQuantityStructure* getGlobalFloatingQuantityStructure(); // creates it if it doesn't exit yet void removeFloatingQuantityStructureIfEmpty();
```
Get global floating quantity structure.

```cpp
void removeFloatingQuantity(std::string name, bool errorIfAbsent = false);
```
Remove floating quantity.

```cpp
void removeAllFloatingQuantities();
```
Remove all floating quantities.

```cpp
template <class T> ScalarImageQuantity* addScalarImageQuantity(std::string name, size_t dimX, size_t dimY, const T& values, ImageOrigin imageOrigin, DataType type = DataType::STANDARD);
```
Add scalar image quantity.

```cpp
template <class T> ColorImageQuantity* addColorImageQuantity(std::string name, size_t dimX, size_t dimY, const T& values_rgb, ImageOrigin imageOrigin);
```
Add color image quantity.

```cpp
template <class T> ColorImageQuantity* addColorAlphaImageQuantity(std::string name, size_t dimX, size_t dimY, const T& values_rgba, ImageOrigin imageOrigin);
```
Add color alpha image quantity.

```cpp
template <class T1, class T2> DepthRenderImageQuantity* addDepthRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& normalData, ImageOrigin imageOrigin);
```
Add depth render image quantity.

```cpp
template <class T1, class T2, class T3> ColorRenderImageQuantity* addColorRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& normalData, const T3& colorData, ImageOrigin imageOrigin);
```
Add color render image quantity.

```cpp
template <class T1, class T2, class T3> ScalarRenderImageQuantity* addScalarRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& normalData, const T3& scalarData, ImageOrigin imageOrigin, DataType type = DataType::STANDARD);
```
Add scalar render image quantity.

```cpp
template <class T1, class T2> RawColorRenderImageQuantity* addRawColorRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& colorData, ImageOrigin imageOrigin);
```
Add raw color render image quantity.

```cpp
template <class T1, class T2> RawColorAlphaRenderImageQuantity* addRawColorAlphaRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& colorData, ImageOrigin imageOrigin);
```
Add raw color alpha render image quantity.

## fullscreen_artist.h

```cpp
class FullscreenArtist : public virtual WeakReferrable {
```
This is a simple class which manages global state amongs Polyscope quanties which draw directly to the whole screen.

```cpp
FullscreenArtist();
```
Fullscreen artist.

```cpp
~FullscreenArtist();
```
Fullscreen artist.

```cpp
FullscreenArtist(const FullscreenArtist&) = delete;
```
Fullscreen artist.

```cpp
FullscreenArtist(FullscreenArtist&&) = delete;
```
Fullscreen artist.

```cpp
FullscreenArtist& operator=(const FullscreenArtist&) = delete;
```
Function.

```cpp
FullscreenArtist& operator=(FullscreenArtist&&) = delete;
```
Function.

```cpp
virtual void disableFullscreenDrawing() = 0;
```
Disable fullscreen drawing.

```cpp
void disableAllFullscreenArtists();
```
Disable all fullscreen artists.

## group.h

```cpp
class Group : public virtual WeakReferrable {
```
Groups track collections of structures (or other groups) which can be toggled together.

```cpp
Group(std::string name);
```
Group.

```cpp
~Group();
```
Group.

```cpp
void buildUI();
```
Build ui.

```cpp
int isEnabled();
```
Return whether enabled.

```cpp
Group* setEnabled(bool newEnabled);
```
Set enabled.

```cpp
void addChildGroup(Group& newChild);
```
Add child group.

```cpp
void addChildStructure(Structure& newChild);
```
Add child structure.

```cpp
void removeChildGroup(Group& child);
```
Remove child group.

```cpp
void removeChildStructure(Structure& child);
```
Remove child structure.

```cpp
void unparent();
```
Unparent.

```cpp
bool isRootGroup();
```
Return whether root group.

```cpp
Group* getTopLevelGrandparent();
```
Get top level grandparent.

```cpp
void appendStructuresToSkip(std::unordered_set<Structure*>& skipSet);
```
Append structures to skip.

```cpp
void appendAllDescendants(std::unordered_set<Structure*>& skipSet);
```
Append all descendants.

```cpp
std::string niceName();
```
Nice name.

```cpp
std::string uniqueName();
```
Unique name.

```cpp
Group* setShowChildDetails(bool newVal);
```
Set show child details.

```cpp
bool getShowChildDetails();
```
Get show child details.

```cpp
Group* setHideDescendantsFromStructureLists(bool newVal);
```
Set hide descendants from structure lists.

```cpp
bool getHideDescendantsFromStructureLists();
```
Get hide descendants from structure lists.

```cpp
WeakHandle<Group> parentGroup;
```
Member parent group.

```cpp
const std::string name;
```
Member name.

```cpp
std::vector<WeakHandle<Group>> childrenGroups;
```
Member children groups.

```cpp
std::vector<WeakHandle<Structure>> childrenStructures;
```
Member children structures.

```cpp
PersistentValue<bool> showChildDetails;
```
Member show child details.

```cpp
PersistentValue<bool> hideDescendantsFromStructureLists;
```
Member hide descendants from structure lists.

```cpp
void cullExpiredChildren();
```
Cull expired children.

## histogram.h

```cpp
class Histogram {
```
A histogram that shows up in ImGUI window ONEDAY: we could definitely make a better histogram widget for categorical data.

```cpp
Histogram();
```
Histogram.

```cpp
Histogram(std::vector<float>& values, DataType datatype);
```
Histogram.

```cpp
~Histogram();
```
Histogram.

```cpp
void buildHistogram(const std::vector<float>& values, DataType datatype);
```
Build histogram.

```cpp
void updateColormap(const std::string& newColormap);
```
Update colormap.

```cpp
void buildUI(float width = -1.0);
```
Build ui.

```cpp
std::pair<double, double> colormapRange;
```
Member double.

```cpp
std::pair<double, double> colormapRange;
```
Member colormap range.

```cpp
void fillBuffers();
```
Fill buffers.

```cpp
DataType dataType = DataType::STANDARD;
```
Member standard.

```cpp
std::vector<float> rawHistCurveY;
```
Member raw hist curve y.

```cpp
std::vector<std::array<float, 2>> rawHistCurveX;
```
Member float.

```cpp
std::vector<std::array<float, 2>> rawHistCurveX;
```
Member raw hist curve x.

```cpp
std::pair<double, double> dataRange;
```
Member double.

```cpp
std::pair<double, double> dataRange;
```
Member data range.

```cpp
void renderToTexture();
```
Render to texture.

```cpp
void prepare();
```
Prepare.

```cpp
std::shared_ptr<render::TextureBuffer> texture = nullptr;
```
Member nullptr.

```cpp
std::shared_ptr<render::FrameBuffer> framebuffer = nullptr;
```
Member nullptr.

```cpp
std::shared_ptr<render::ShaderProgram> program = nullptr;
```
Member nullptr.

## image_quantity.h

_No API symbols detected._

## image_quantity_base.h

```cpp
class CameraView;
```
forward declaration since it appears as a class member below

```cpp
class ImageQuantity : public FloatingQuantity, public FullscreenArtist {
```
Class ImageQuantity.

```cpp
ImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, ImageOrigin imageOrigin);
```
Image quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void disableFullscreenDrawing() override;
```
Disable fullscreen drawing.

```cpp
Structure& parent;
```
Member parent.

```cpp
size_t nPix();
```
N pix.

```cpp
void setShowFullscreen(bool newVal);
```
Set show fullscreen.

```cpp
bool getShowFullscreen();
```
Get show fullscreen.

```cpp
void setShowInImGuiWindow(bool newVal);
```
Set show in im gui window.

```cpp
bool getShowInImGuiWindow();
```
Get show in im gui window.

```cpp
void setShowInCameraBillboard(bool newVal);
```
Set show in camera billboard.

```cpp
bool getShowInCameraBillboard();
```
Get show in camera billboard.

```cpp
void setTransparency(float newVal);
```
Set transparency.

```cpp
float getTransparency();
```
Get transparency.

```cpp
const size_t dimX, dimY;
```
Member dim x.

```cpp
const size_t dimX, dimY;
```
Member dim y.

```cpp
ImageOrigin imageOrigin;
```
Member image origin.

```cpp
PersistentValue<float> transparency;
```
Member transparency.

```cpp
PersistentValue<bool> isShowingFullscreen, isShowingImGuiWindow, isShowingCameraBillboard;
```
Member is showing fullscreen.

```cpp
PersistentValue<bool> isShowingFullscreen, isShowingImGuiWindow, isShowingCameraBillboard;
```
Member is showing im gui window.

```cpp
PersistentValue<bool> isShowingFullscreen, isShowingImGuiWindow, isShowingCameraBillboard;
```
Member is showing camera billboard.

```cpp
CameraView* parentStructureCameraView = nullptr;
```
Member nullptr.

```cpp
virtual void showFullscreen() = 0;
```
Show fullscreen.

```cpp
virtual void showInImGuiWindow() = 0;
```
Show in im gui window.

```cpp
virtual void showInBillboard(glm::vec3 center, glm::vec3 upVec, glm::vec3 rightVec) = 0;
```
Show in billboard.

```cpp
virtual void renderIntermediate();
```
Render intermediate.

```cpp
bool parentIsCameraView();
```
Parent is camera view.

```cpp
void buildImageUI();
```
Build image ui.

```cpp
void buildImageOptionsUI();
```
Build image options ui.

## imgui_config.h

```cpp
void configureImGuiStyle();
```
Configure im gui style.

```cpp
std::tuple<ImFontAtlas*, ImFont*, ImFont*> prepareImGuiFonts();
```
Prepare im gui fonts.

## implicit_helpers.h

```cpp
struct ImplicitRenderOpts {
```
A collection of helper functions for generating visualizations of implicitly-defined data (that is, where you have a function that you can evaluate at f(x,y,z) to get back a scalar, color, etc.

```cpp
CameraParameters cameraParameters = CameraParameters::createInvalid();
```
Create invalid.

```cpp
ScaledValue<float> missDist = ScaledValue<float>::relative(20.);
```
Relative.

```cpp
ScaledValue<float> hitDist = ScaledValue<float>::relative(1e-4);
```
Relative.

```cpp
ScaledValue<float> stepSize = ScaledValue<float>::relative(1e-2);
```
Relative.

```cpp
template <class S> void resolveImplicitRenderOpts(QuantityStructure<S>* parent, ImplicitRenderOpts& opts);
```
Resolve implicit render opts.

```cpp
template <class S> template <class Func, class S> DepthRenderImageQuantity* renderImplicitSurface(QuantityStructure<S>* parent, std::string name, Func&& func, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface.

```cpp
template <class Func> DepthRenderImageQuantity* renderImplicitSurface(std::string name, Func&& func, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface.

```cpp
template <class Func, class S> DepthRenderImageQuantity* renderImplicitSurfaceBatch(QuantityStructure<S>* parent, std::string name, Func&& func, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface batch.

```cpp
template <class Func> DepthRenderImageQuantity* renderImplicitSurfaceBatch(std::string name, Func&& func, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface batch.

```cpp
template <class Func, class FuncColor, class S> ColorRenderImageQuantity* renderImplicitSurfaceColor(QuantityStructure<S>* parent, std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface color.

```cpp
template <class Func, class FuncColor> ColorRenderImageQuantity* renderImplicitSurfaceColor(std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface color.

```cpp
template <class Func, class FuncColor, class S> ColorRenderImageQuantity* renderImplicitSurfaceColorBatch(QuantityStructure<S>* parent, std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface color batch.

```cpp
template <class Func, class FuncColor> ColorRenderImageQuantity* renderImplicitSurfaceColorBatch(std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface color batch.

```cpp
template <class Func, class FuncScalar, class S> ScalarRenderImageQuantity* renderImplicitSurfaceScalar(QuantityStructure<S>* parent, std::string name, Func&& func, FuncScalar&& funcScalar, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts(), DataType dataType = DataType::STANDARD);
```
Render implicit surface scalar.

```cpp
template <class Func, class FuncScalar> ScalarRenderImageQuantity* renderImplicitSurfaceScalar(std::string name, Func&& func, FuncScalar&& funcScalar, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts(), DataType dataType = DataType::STANDARD);
```
Render implicit surface scalar.

```cpp
template <class Func, class FuncScalar, class S> ScalarRenderImageQuantity* renderImplicitSurfaceScalarBatch(QuantityStructure<S>* parent, std::string name, Func&& func, FuncScalar&& funcScalar, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts(), DataType dataType = DataType::STANDARD);
```
Render implicit surface scalar batch.

```cpp
template <class Func, class FuncScalar> ScalarRenderImageQuantity* renderImplicitSurfaceScalarBatch(std::string name, Func&& func, FuncScalar&& funcScalar, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts(), DataType dataType = DataType::STANDARD);
```
Render implicit surface scalar batch.

```cpp
template <class Func, class FuncColor, class S> RawColorRenderImageQuantity* renderImplicitSurfaceRawColor(QuantityStructure<S>* parent, std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface raw color.

```cpp
template <class Func, class FuncColor> RawColorRenderImageQuantity* renderImplicitSurfaceRawColor(std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface raw color.

```cpp
template <class Func, class FuncColor, class S> RawColorRenderImageQuantity* renderImplicitSurfaceRawColorBatch(QuantityStructure<S>* parent, std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface raw color batch.

```cpp
template <class Func, class FuncColor> RawColorRenderImageQuantity* renderImplicitSurfaceRawColorBatch(std::string name, Func&& func, FuncColor&& funcColor, ImplicitRenderMode mode, ImplicitRenderOpts opts = ImplicitRenderOpts());
```
Render implicit surface raw color batch.

## internal.h

```cpp
class FloatingQuantityStructure;
```
forward declaration

```cpp
uint64_t getNextUniqueID();
```
Get next unique id.

## messages.h

```cpp
void info(std::string message); // default verbosityLevel = 0 void info(int verbosityLevel, std::string message); // only printed if verbosity > vebosityLevel // Non-fatal warnings. Warnings with the same base message are batched together, so the UI doesn't get completely // overwhelmed if you call this in a dense loop. void warning(std::string baseMessage, std::string detailMessage = "");
```
Info.

```cpp
void error(std::string message);
```
Error.

```cpp
void terminatingError(std::string message);
```
Terminating error.

```cpp
void exception(std::string message);
```
Exception.

```cpp
void showDelayedWarnings();
```
Show delayed warnings.

```cpp
void clearMessages();
```
Clear messages.

## numeric_helpers.h

```cpp
template <typename T> bool allComponentsFinite(const T& x) { // handle all other scalar types by converting to float, it's what we'll do anyway return std::isfinite(static_cast<float>(x));
```
All components finite.

```cpp
template <> inline bool allComponentsFinite<glm::vec2>(const glm::vec2& x) { return glm::all(glm::isfinite(x));
```
All.

```cpp
template <> inline bool allComponentsFinite<glm::vec3>(const glm::vec3& x) { return glm::all(glm::isfinite(x));
```
All.

```cpp
template <> inline bool allComponentsFinite<glm::vec4>(const glm::vec4& x) { return glm::all(glm::isfinite(x));
```
All.

```cpp
template <> inline bool allComponentsFinite<glm::mat2x2>(const glm::mat2x2& x) { for (size_t i = 0; i < 2; i++) if (!allComponentsFinite(glm::row(x, i))) return false;
```
For.

```cpp
template <> inline bool allComponentsFinite<glm::mat3x3>(const glm::mat3x3& x) { for (size_t i = 0; i < 3; i++) if (!allComponentsFinite(glm::row(x, i))) return false;
```
For.

```cpp
template <> inline bool allComponentsFinite<glm::mat4x4>(const glm::mat4x4& x) { for (size_t i = 0; i < 4; i++) if (!allComponentsFinite(glm::row(x, i))) return false;
```
For.

```cpp
template <> inline bool allComponentsFinite<std::array<glm::vec3, 2>>(const std::array<glm::vec3, 2>& x) { for (size_t i = 0; i < x.size(); i++) if (!glm::all(glm::isfinite(x[i]))) return false;
```
For.

```cpp
template <> inline bool allComponentsFinite<std::array<glm::vec3, 3>>(const std::array<glm::vec3, 3>& x) { for (size_t i = 0; i < x.size(); i++) if (!glm::all(glm::isfinite(x[i]))) return false;
```
For.

```cpp
template <> inline bool allComponentsFinite<std::array<glm::vec3, 4>>(const std::array<glm::vec3, 4>& x) { for (size_t i = 0; i < x.size(); i++) if (!glm::all(glm::isfinite(x[i]))) return false;
```
For.

## options.h

_No API symbols detected._

## parameterization_quantity.h

```cpp
template <typename QuantityT> class ParameterizationQuantity {
```
Class ParameterizationQuantity.

```cpp
ParameterizationQuantity(QuantityT& quantity, const std::vector<glm::vec2>& coords_, ParamCoordsType type_, ParamVizStyle style_);
```
Parameterization quantity.

```cpp
void buildParameterizationUI();
```
Build parameterization ui.

```cpp
virtual void buildParameterizationOptionsUI();
```
Build parameterization options ui.

```cpp
template <class V> void updateCoords(const V& newCoords);
```
Update coords.

```cpp
QuantityT& quantity;
```
Member quantity.

```cpp
render::ManagedBuffer<glm::vec2> coords;
```
Member coords.

```cpp
render::ManagedBuffer<float> islandLabels;
```
Member island labels.

```cpp
const ParamCoordsType coordsType;
```
Member coords type.

```cpp
QuantityT* setStyle(ParamVizStyle newStyle);
```
Set style.

```cpp
ParamVizStyle getStyle();
```
Get style.

```cpp
QuantityT* setCheckerColors(std::pair<glm::vec3, glm::vec3> colors);
```
Set checker colors.

```cpp
std::pair<glm::vec3, glm::vec3> getCheckerColors();
```
Get checker colors.

```cpp
QuantityT* setGridColors(std::pair<glm::vec3, glm::vec3> colors);
```
Set grid colors.

```cpp
std::pair<glm::vec3, glm::vec3> getGridColors();
```
Get grid colors.

```cpp
QuantityT* setCheckerSize(double newVal);
```
Set checker size.

```cpp
double getCheckerSize();
```
Get checker size.

```cpp
QuantityT* setColorMap(std::string val);
```
Set color map.

```cpp
std::string getColorMap();
```
Get color map.

```cpp
QuantityT* setAltDarkness(double newVal);
```
Set alt darkness.

```cpp
double getAltDarkness();
```
Get alt darkness.

```cpp
std::vector<std::string> addParameterizationRules(std::vector<std::string> rules);
```
Add parameterization rules.

```cpp
void fillParameterizationBuffers(render::ShaderProgram& p);
```
Fill parameterization buffers.

```cpp
void setParameterizationUniforms(render::ShaderProgram& p);
```
Set parameterization uniforms.

```cpp
std::vector<glm::vec2> coordsData;
```
Member coords data.

```cpp
std::vector<float> islandLabelsData;
```
Member island labels data.

```cpp
bool islandLabelsPopulated = false;
```
Member false.

```cpp
PersistentValue<float> checkerSize;
```
Member checker size.

```cpp
PersistentValue<ParamVizStyle> vizStyle;
```
Member viz style.

```cpp
PersistentValue<glm::vec3> checkColor1, checkColor2;
```
Member check color 1.

```cpp
PersistentValue<glm::vec3> checkColor1, checkColor2;
```
Member check color 2.

```cpp
PersistentValue<glm::vec3> gridLineColor, gridBackgroundColor;
```
Member grid line color.

```cpp
PersistentValue<glm::vec3> gridLineColor, gridBackgroundColor;
```
Member grid background color.

```cpp
PersistentValue<float> altDarkness;
```
Member alt darkness.

```cpp
PersistentValue<std::string> cMap;
```
Member c map.

```cpp
bool haveIslandLabels();
```
Have island labels.

## persistent_value.h

```cpp
template <typename T> class PersistentCache {
```
Class PersistentCache.

```cpp
std::unordered_map<std::string, T> cache;
```
Member string.

```cpp
std::unordered_map<std::string, T> cache;
```
Member cache.

```cpp
template <typename T> PersistentCache<T>& getPersistentCacheRef();
```
Get persistent cache ref.

```cpp
template <typename T> } // namespace detail template <typename T> class PersistentValue {
```
Class PersistentValue.

```cpp
PersistentValue(const std::string& name_, T value_) : name(name_), value(value_) { if (detail::getPersistentCacheRef<T>().cache.find(name) != detail::getPersistentCacheRef<T>().cache.end()) { value = detail::getPersistentCacheRef<T>().cache[name];
```
Persistent value.

```cpp
holdsDefaultValue_ = false;
```
Member false.

```cpp
} else { detail::getPersistentCacheRef<T>().cache[name] = value;
```
Function.

```cpp
} } ~PersistentValue() {} PersistentValue(const PersistentValue&) = delete;
```
Persistent value.

```cpp
PersistentValue(const PersistentValue&&) = delete;
```
Persistent value.

```cpp
template <typename U> PersistentValue<T>& operator=(const PersistentValue<U>& other) { set(other.value);
```
Set .

```cpp
return *this;
```
Member this.

```cpp
} template <typename U> PersistentValue<T>& operator=(const PersistentValue<U>&& other) { set(other.value);
```
Set .

```cpp
return *this;
```
Member this.

```cpp
} template <typename U> PersistentValue<T>& operator=(const U& value_) { set(value_);
```
Set .

```cpp
return *this;
```
Member this.

```cpp
} template <typename U> PersistentValue<T>& operator=(const U&& value_) { set(value_);
```
Set .

```cpp
return *this;
```
Member this.

```cpp
} T& get() { return value; }
```
Get .

```cpp
void manuallyChanged() { set(value); }
```
Manually changed.

```cpp
void clearCache() { detail::getPersistentCacheRef<T>().cache.erase(name);
```
Clear cache.

```cpp
holdsDefaultValue_ = true;
```
Member true.

```cpp
} void set(T value_) { value = value_;
```
Set .

```cpp
detail::getPersistentCacheRef<T>().cache[name] = value;
```
Function.

```cpp
holdsDefaultValue_ = false;
```
Member false.

```cpp
} void setPassive(T value_) { if (holdsDefaultValue_) { value = value_;
```
Set passive.

```cpp
detail::getPersistentCacheRef<T>().cache[name] = value;
```
Function.

```cpp
} } bool holdsDefaultValue() const { return holdsDefaultValue_; }
```
Holds default value.

```cpp
template <typename> friend class PersistentValue;
```
Member persistent value.

```cpp
const std::string name;
```
Member name.

```cpp
T value;
```
Member value.

```cpp
bool holdsDefaultValue_ = true;
```
Member true.

## pick.h

```cpp
class Structure;
```
Forward decls

```cpp
struct PickResult {
```
== Main query Pick queries test a screen location in the rendered viewport, and return a variety of info about what is underneath the pixel at that point, including what structure is under the cursor, and the scene depth and color.

```cpp
bool isHit = false;
```
Member false.

```cpp
Structure* structure = nullptr;
```
Member nullptr.

```cpp
WeakHandle<Structure> structureHandle;
```
Member structure handle.

```cpp
glm::vec2 screenCoords;
```
Member screen coords.

```cpp
glm::ivec2 bufferInds;
```
Member buffer inds.

```cpp
glm::vec3 position;
```
Member position.

```cpp
float depth;
```
Member depth.

```cpp
uint64_t localIndex = INVALID_IND_64;
```
Member invalid ind 64.

```cpp
PickResult pickAtScreenCoords(glm::vec2 screenCoords); // takes screen coordinates PickResult pickAtBufferInds(glm::ivec2 bufferInds); // takes indices into render buffer // == Stateful picking: track and update a current selection // Get/Set the "selected" item, if there is one PickResult getSelection();
```
Pick at screen coords.

```cpp
void setSelection(PickResult newPick);
```
Set selection.

```cpp
void resetSelection();
```
Reset selection.

```cpp
bool haveSelection();
```
Have selection.

```cpp
void resetSelectionIfStructure(Structure* s); // If something from this structure is selected, clear the selection // (useful if a structure is being deleted) namespace pick { // Old, deprecated picking API. Use the above functions instead. // Get the structure which was clicked on (nullptr if none), and the pick ID in local indices for that structure (such // that 0 is the first index as returned from requestPickBufferRange()) std::pair<Structure*, uint64_t> pickAtScreenCoords(glm::vec2 screenCoords); // takes screen coordinates std::pair<Structure*, uint64_t> pickAtBufferCoords(int xPos, int yPos); // takes indices into the buffer std::pair<Structure*, uint64_t> evaluatePickQuery(int xPos, int yPos); // old, badly named. takes buffer coordinates. // == Helpers // Set up picking (internal) // Called by a structure to figure out what data it should render to the pick buffer. // Request 'count' contiguous indices for drawing a pick buffer. The return value is the start of the range. uint64_t requestPickBufferRange(Structure* requestingStructure, uint64_t count);
```
Reset selection if structure.

```cpp
std::pair<Structure*, uint64_t> globalIndexToLocal(uint64_t globalInd);
```
Global index to local.

```cpp
uint64_t localIndexToGlobal(std::pair<Structure*, uint64_t> localPick);
```
Local index to global.

```cpp
inline glm::vec3 indToVec(uint64_t globalInd);
```
Ind to vec.

```cpp
inline uint64_t vecToInd(glm::vec3 vec);
```
Vec to ind.

## point_cloud.h

```cpp
class PointCloud;
```
Forward declare point cloud

```cpp
class PointCloudColorQuantity;
```
Forward declare quantity types

```cpp
class PointCloudScalarQuantity;
```
Class PointCloudScalarQuantity.

```cpp
class PointCloudParameterizationQuantity;
```
Class PointCloudParameterizationQuantity.

```cpp
class PointCloudVectorQuantity;
```
Class PointCloudVectorQuantity.

```cpp
template <> // Specialize the quantity type struct QuantityTypeHelper<PointCloud> {
```
Struct QuantityTypeHelper.

```cpp
struct PointCloudPickResult {
```
Struct PointCloudPickResult.

```cpp
int64_t index;
```
Member index.

```cpp
class PointCloud : public QuantityStructure<PointCloud> {
```
Class PointCloud.

```cpp
PointCloud(std::string name, std::vector<glm::vec3> points);
```
Point cloud.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void buildPickUI(const PickResult& result) override;
```
Build pick ui.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
render::ManagedBuffer<glm::vec3> points;
```
Member points.

```cpp
template <class T> PointCloudScalarQuantity* addScalarQuantity(std::string name, const T& values, DataType type = DataType::STANDARD);
```
Add scalar quantity.

```cpp
template <class T> PointCloudParameterizationQuantity* addParameterizationQuantity(std::string name, const T& values, ParamCoordsType type = ParamCoordsType::UNIT);
```
Add parameterization quantity.

```cpp
template <class T> PointCloudParameterizationQuantity* addLocalParameterizationQuantity(std::string name, const T& values, ParamCoordsType type = ParamCoordsType::WORLD);
```
Add local parameterization quantity.

```cpp
template <class T> PointCloudColorQuantity* addColorQuantity(std::string name, const T& values);
```
Add color quantity.

```cpp
template <class T> PointCloudVectorQuantity* addVectorQuantity(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add vector quantity.

```cpp
template <class T> PointCloudVectorQuantity* addVectorQuantity2D(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add vector quantity 2 d.

```cpp
template <class V> void updatePointPositions(const V& newPositions);
```
Update point positions.

```cpp
template <class V> void updatePointPositions2D(const V& newPositions);
```
Update point positions 2 d.

```cpp
void setPointRadiusQuantity(PointCloudScalarQuantity* quantity, bool autoScale = true);
```
Set point radius quantity.

```cpp
void setPointRadiusQuantity(std::string name, bool autoScale = true);
```
Set point radius quantity.

```cpp
void clearPointRadiusQuantity();
```
Clear point radius quantity.

```cpp
void setTransparencyQuantity(PointCloudScalarQuantity* quantity);
```
Set transparency quantity.

```cpp
void setTransparencyQuantity(std::string name);
```
Set transparency quantity.

```cpp
void clearTransparencyQuantity();
```
Clear transparency quantity.

```cpp
size_t nPoints();
```
N points.

```cpp
glm::vec3 getPointPosition(size_t iPt);
```
Get point position.

```cpp
PointCloudPickResult interpretPickResult(const PickResult& result);
```
Interpret pick result.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
void deleteProgram();
```
Delete program.

```cpp
PointCloud* setPointRenderMode(PointRenderMode newVal);
```
Set point render mode.

```cpp
PointRenderMode getPointRenderMode();
```
Get point render mode.

```cpp
PointCloud* setPointColor(glm::vec3 newVal);
```
Set point color.

```cpp
glm::vec3 getPointColor();
```
Get point color.

```cpp
PointCloud* setPointRadius(double newVal, bool isRelative = true);
```
Set point radius.

```cpp
double getPointRadius();
```
Get point radius.

```cpp
PointCloud* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
void setPointCloudUniforms(render::ShaderProgram& p);
```
Set point cloud uniforms.

```cpp
void setPointProgramGeometryAttributes(render::ShaderProgram& p);
```
Set point program geometry attributes.

```cpp
std::vector<std::string> addPointCloudRules(std::vector<std::string> initRules, bool withPointCloud = true);
```
Add point cloud rules.

```cpp
std::string getShaderNameForRenderMode();
```
Get shader name for render mode.

```cpp
std::vector<glm::vec3> pointsData;
```
Member points data.

```cpp
PersistentValue<std::string> pointRenderMode;
```
Member point render mode.

```cpp
PersistentValue<glm::vec3> pointColor;
```
Member point color.

```cpp
PersistentValue<ScaledValue<float>> pointRadius;
```
Member point radius.

```cpp
PersistentValue<std::string> material;
```
Member material.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
std::shared_ptr<render::ShaderProgram> pickProgram;
```
Member pick program.

```cpp
void ensureRenderProgramPrepared();
```
Ensure render program prepared.

```cpp
void ensurePickProgramPrepared();
```
Ensure pick program prepared.

```cpp
PointCloudScalarQuantity* addScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add scalar quantity impl.

```cpp
PointCloudParameterizationQuantity* addParameterizationQuantityImpl(std::string name, const std::vector<glm::vec2>& param, ParamCoordsType type);
```
Add parameterization quantity impl.

```cpp
PointCloudParameterizationQuantity* addLocalParameterizationQuantityImpl(std::string name, const std::vector<glm::vec2>& param, ParamCoordsType type);
```
Add local parameterization quantity impl.

```cpp
PointCloudColorQuantity* addColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
```
Add color quantity impl.

```cpp
PointCloudVectorQuantity* addVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors, VectorType vectorType);
```
Add vector quantity impl.

```cpp
bool pointRadiusQuantityAutoscale = true;
```
Member true.

```cpp
PointCloudScalarQuantity& resolvePointRadiusQuantity();
```
Resolve point radius quantity.

```cpp
PointCloudScalarQuantity& resolveTransparencyQuantity();
```
Resolve transparency quantity.

```cpp
template <class T> PointCloud* registerPointCloud(std::string name, const T& points);
```
Register point cloud.

```cpp
template <class T> template <class T> PointCloud* registerPointCloud2D(std::string name, const T& points);
```
Register point cloud 2 d.

```cpp
inline PointCloud* getPointCloud(std::string name = "");
```
Get point cloud.

```cpp
inline bool hasPointCloud(std::string name = "");
```
Return whether it has point cloud.

```cpp
inline void removePointCloud(std::string name = "", bool errorIfAbsent = false);
```
Remove point cloud.

## point_cloud_color_quantity.h

```cpp
class PointCloudColorQuantity : public PointCloudQuantity, public ColorQuantity<PointCloudColorQuantity> {
```
Class PointCloudColorQuantity.

```cpp
PointCloudColorQuantity(std::string name, const std::vector<glm::vec3>& values, PointCloud& pointCloud_);
```
Point cloud color quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildPickUI(size_t ind) override;
```
Build pick ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
void createPointProgram();
```
Create point program.

```cpp
std::shared_ptr<render::ShaderProgram> pointProgram;
```
Member point program.

## point_cloud_parameterization_quantity.h

```cpp
class PointCloudParameterizationQuantity : public PointCloudQuantity, class PointCloudParameterizationQuantity : public PointCloudQuantity, public ParameterizationQuantity<PointCloudParameterizationQuantity> {
```
Class PointCloudParameterizationQuantity.

```cpp
PointCloudParameterizationQuantity(std::string name, PointCloud& cloud_, const std::vector<glm::vec2>& coords_, ParamCoordsType type_, ParamVizStyle style_);
```
Point cloud parameterization quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildPickUI(size_t ind) override;
```
Build pick ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
void createProgram();
```
Create program.

```cpp
void fillCoordBuffers(render::ShaderProgram& p);
```
Fill coord buffers.

## point_cloud_quantity.h

```cpp
class PointCloud;
```
Forward delcare point cloud

```cpp
class PointCloudQuantity : public QuantityS<PointCloud> {
```
Extend Quantity<PointCloud> to add a few extra functions

```cpp
PointCloudQuantity(std::string name, PointCloud& parentStructure, bool dominates = false);
```
Point cloud quantity.

```cpp
virtual ~PointCloudQuantity() {};
```
Point cloud quantity.

```cpp
virtual void buildInfoGUI(size_t pointInd);
```
Build info gui.

## point_cloud_scalar_quantity.h

```cpp
class PointCloudScalarQuantity : public PointCloudQuantity, public ScalarQuantity<PointCloudScalarQuantity> {
```
Class PointCloudScalarQuantity.

```cpp
PointCloudScalarQuantity(std::string name, const std::vector<float>& values, PointCloud& pointCloud_, DataType dataType);
```
Point cloud scalar quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildPickUI(size_t ind) override;
```
Build pick ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
void createProgram();
```
Create program.

```cpp
std::shared_ptr<render::ShaderProgram> pointProgram;
```
Member point program.

## point_cloud_vector_quantity.h

```cpp
class PointCloudVectorQuantity : public PointCloudQuantity, public VectorQuantity<PointCloudVectorQuantity> {
```
Represents a general vector field associated with a point cloud

```cpp
PointCloudVectorQuantity(std::string name, std::vector<glm::vec3> vectors, PointCloud& pointCloud_, VectorType vectorType_ = VectorType::STANDARD);
```
Point cloud vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildPickUI(size_t ind) override;
```
Build pick ui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

## polyscope.h

```cpp
class Structure;
```
forward declarations

```cpp
class Group;
```
Class Group.

```cpp
class SlicePlane;
```
Class SlicePlane.

```cpp
class Widget;
```
Class Widget.

```cpp
void init(std::string backend = "");
```
Init.

```cpp
void checkInitialized();
```
Check initialized.

```cpp
bool isInitialized();
```
Return whether initialized.

```cpp
void show(size_t forFrames = std::numeric_limits<size_t>::max());
```
Show.

```cpp
void unshow();
```
Unshow.

```cpp
void frameTick();
```
Frame tick.

```cpp
void shutdown(bool allowMidFrameShutdown = false);
```
Shutdown.

```cpp
bool windowRequestsClose();
```
Window requests close.

```cpp
bool isHeadless();
```
Return whether headless.

```cpp
glm::vec3 center();
```
Center.

```cpp
} // namespace state Structure* getStructure(std::string type, std::string name = "");
```
Get structure.

```cpp
bool hasStructure(std::string type, std::string name);
```
Return whether it has structure.

```cpp
std::tuple<std::string, std::string> lookUpStructure(Structure* structure);
```
Look up structure.

```cpp
void removeStructure(Structure* structure, bool errorIfAbsent = false);
```
Remove structure.

```cpp
void removeStructure(std::string type, std::string name, bool errorIfAbsent = false);
```
Remove structure.

```cpp
void removeStructure(std::string name, bool errorIfAbsent = false);
```
Remove structure.

```cpp
void removeAllStructures();
```
Remove all structures.

```cpp
void updateStructureExtents();
```
Update structure extents.

```cpp
Group* createGroup(std::string name);
```
Create group.

```cpp
Group* getGroup(std::string name);
```
Get group.

```cpp
void removeGroup(Group* group, bool errorIfAbsent = true);
```
Remove group.

```cpp
void removeGroup(std::string name, bool errorIfAbsent = true);
```
Remove group.

```cpp
void removeAllGroups();
```
Remove all groups.

```cpp
void refresh();
```
Refresh.

```cpp
void draw(bool withUI = true, bool withContextCallback = true);
```
Draw.

```cpp
void requestRedraw();
```
Request redraw.

```cpp
bool redrawRequested();
```
Redraw requested.

```cpp
void pushContext(std::function<void()> callbackFunction, bool drawDefaultUI = true);
```
Push context.

```cpp
void popContext();
```
Pop context.

```cpp
ImGuiContext* getCurrentContext();
```
Get current context.

```cpp
void buildPolyscopeGui();
```
Build polyscope gui.

```cpp
void buildStructureGui();
```
Build structure gui.

```cpp
void buildPickGui();
```
Build pick gui.

```cpp
void buildUserGuiAndInvokeCallback();
```
Build user gui and invoke callback.

```cpp
void mainLoopIteration();
```
Main loop iteration.

```cpp
void initializeImGUIContext();
```
Initialize im gui context.

```cpp
void drawStructures();
```
Draw structures.

```cpp
void drawStructuresDelayed();
```
Draw structures delayed.

```cpp
void processLazyProperties();
```
Process lazy properties.

```cpp
void processLazyPropertiesOutsideOfImGui();
```
Process lazy properties outside of im gui.

## quantity.h

```cpp
class Structure;
```
A 'quantity' (in Polyscope terminology) is data which is associated with a structure; any structure might have many quantities.

```cpp
class Quantity : public render::ManagedBufferRegistry {
```
=== General Quantities (subclasses could be a structure-specific quantity or a floating quantity)

```cpp
Quantity(std::string name, Structure& parentStructure);
```
Quantity.

```cpp
virtual ~Quantity();
```
Quantity.

```cpp
virtual void draw();
```
Draw.

```cpp
virtual void drawDelayed();
```
Draw delayed.

```cpp
virtual void buildUI();
```
Build ui.

```cpp
virtual void buildCustomUI();
```
Build custom ui.

```cpp
virtual void buildPickUI(size_t localPickInd);
```
Build pick ui.

```cpp
bool isEnabled();
```
Return whether enabled.

```cpp
virtual void refresh();
```
Refresh.

```cpp
virtual std::string niceName();
```
Nice name.

```cpp
std::string uniquePrefix();
```
Unique prefix.

```cpp
Structure& parent;
```
Member parent.

```cpp
const std::string name;
```
Member name.

```cpp
PersistentValue<bool> enabled;
```
Member enabled.

```cpp
template <typename S> class QuantityS : public Quantity {
```
Class QuantityS.

```cpp
QuantityS(std::string name, S& parentStructure, bool dominates = false);
```
Quantity s.

```cpp
virtual ~QuantityS();
```
Quantity s.

```cpp
virtual QuantityS<S>* setEnabled(bool newEnabled);
```
Set enabled.

```cpp
virtual void buildUI() override;
```
Build ui.

```cpp
S& parent;
```
Member parent.

```cpp
bool dominates = false;
```
Member false.

## ragged_nested_array.h

_No API symbols detected._

## raw_color_alpha_render_image_quantity.h

```cpp
class RawColorAlphaRenderImageQuantity : public RenderImageQuantityBase {
```
Class RawColorAlphaRenderImageQuantity.

```cpp
RawColorAlphaRenderImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec4>& colorsData, ImageOrigin imageOrigin);
```
Raw color alpha render image quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
render::ManagedBuffer<glm::vec4> colors;
```
Member colors.

```cpp
template <typename T1, typename T2> void updateBuffers(const T1& depthData, const T2& colorsData);
```
Update buffers.

```cpp
RawColorAlphaRenderImageQuantity* setIsPremultiplied(bool val);
```
Set is premultiplied.

```cpp
bool getIsPremultiplied();
```
Get is premultiplied.

```cpp
std::vector<glm::vec4> colorsData;
```
Member colors data.

```cpp
PersistentValue<bool> isPremultiplied;
```
Member is premultiplied.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
void prepare();
```
Prepare.

```cpp
template <typename T1, typename T2> void RawColorAlphaRenderImageQuantity::updateBuffers(const T1& depthData, const T2& colorsData) { validateSize(depthData, dimX * dimY, "color render image depth data " + name);
```
Update buffers.

```cpp
validateSize(colorsData, dimX * dimY, "color render image color data " + name);
```
Validate size.

```cpp
std::vector<float> standardDepth(standardizeArray<float>(depthData));
```
Standard depth.

```cpp
std::vector<glm::vec4> standardColor(standardizeVectorArray<glm::vec4, 4>(colorsData));
```
Standard color.

```cpp
colors.markHostBufferUpdated();
```
Mark host buffer updated.

```cpp
updateBaseBuffers(standardDepth, standardNormal);
```
Update base buffers.

## raw_color_render_image_quantity.h

```cpp
class RawColorRenderImageQuantity : public RenderImageQuantityBase {
```
Class RawColorRenderImageQuantity.

```cpp
RawColorRenderImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& colorsData, ImageOrigin imageOrigin);
```
Raw color render image quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
render::ManagedBuffer<glm::vec3> colors;
```
Member colors.

```cpp
template <typename T1, typename T2> void updateBuffers(const T1& depthData, const T2& colorsData);
```
Update buffers.

```cpp
std::vector<glm::vec3> colorsData;
```
Member colors data.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
void prepare();
```
Prepare.

```cpp
template <typename T1, typename T2> void RawColorRenderImageQuantity::updateBuffers(const T1& depthData, const T2& colorsData) { validateSize(depthData, dimX * dimY, "color render image depth data " + name);
```
Update buffers.

```cpp
validateSize(colorsData, dimX * dimY, "color render image color data " + name);
```
Validate size.

```cpp
std::vector<float> standardDepth(standardizeArray<float>(depthData));
```
Standard depth.

```cpp
std::vector<glm::vec3> standardColor(standardizeVectorArray<glm::vec3, 3>(colorsData));
```
Standard color.

```cpp
colors.markHostBufferUpdated();
```
Mark host buffer updated.

```cpp
updateBaseBuffers(standardDepth, standardNormal);
```
Update base buffers.

## render_image_quantity_base.h

```cpp
* Base class for RenderImage classes, which are render-like data of onscreen geometry (buffers of depth, normals, etc) * which has been generated from some out-of-Polyscope process, and is to be rendered to the screen. * */ namespace polyscope { class RenderImageQuantityBase : public FloatingQuantity, public FullscreenArtist { public: RenderImageQuantityBase(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& normalData, ImageOrigin imageOrigin);
```
Geometry.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
size_t nPix();
```
N pix.

```cpp
void updateBaseBuffers(const std::vector<float>& newDepthData, const std::vector<glm::vec3>& newNormalData);
```
Update base buffers.

```cpp
virtual void disableFullscreenDrawing() override;
```
Disable fullscreen drawing.

```cpp
virtual RenderImageQuantityBase* setEnabled(bool newEnabled) override;
```
Set enabled.

```cpp
RenderImageQuantityBase* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
RenderImageQuantityBase* setTransparency(float newVal);
```
Set transparency.

```cpp
float getTransparency();
```
Get transparency.

```cpp
RenderImageQuantityBase* setAllowFullscreenCompositing(bool newVal);
```
Set allow fullscreen compositing.

```cpp
bool getAllowFullscreenCompositing();
```
Get allow fullscreen compositing.

```cpp
void prepareGeometryBuffers();
```
Prepare geometry buffers.

```cpp
void addOptionsPopupEntries();
```
Add options popup entries.

## scalar_image_quantity.h

```cpp
class ScalarImageQuantity : public ImageQuantity, public ScalarQuantity<ScalarImageQuantity> {
```
Class ScalarImageQuantity.

```cpp
ScalarImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<float>& data, ImageOrigin imageOrigin, DataType dataType);
```
Scalar image quantity.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual ScalarImageQuantity* setEnabled(bool newEnabled) override;
```
Set enabled.

```cpp
std::shared_ptr<render::TextureBuffer> textureIntermediateRendered;
```
Member texture intermediate rendered.

```cpp
std::shared_ptr<render::ShaderProgram> fullscreenProgram, billboardProgram;
```
Member fullscreen program.

```cpp
std::shared_ptr<render::ShaderProgram> fullscreenProgram, billboardProgram;
```
Member billboard program.

```cpp
std::shared_ptr<render::FrameBuffer> framebufferIntermediate;
```
Member framebuffer intermediate.

```cpp
void prepareFullscreen();
```
Prepare fullscreen.

```cpp
void prepareIntermediateRender();
```
Prepare intermediate render.

```cpp
void prepareBillboard();
```
Prepare billboard.

```cpp
virtual void showFullscreen() override;
```
Show fullscreen.

```cpp
virtual void showInImGuiWindow() override;
```
Show in im gui window.

```cpp
virtual void showInBillboard(glm::vec3 center, glm::vec3 upVec, glm::vec3 rightVec) override;
```
Show in billboard.

```cpp
virtual void renderIntermediate() override;
```
Render intermediate.

## scalar_quantity.h

```cpp
template <typename QuantityT> class ScalarQuantity {
```
Class ScalarQuantity.

```cpp
ScalarQuantity(QuantityT& quantity, const std::vector<float>& values, DataType dataType);
```
Scalar quantity.

```cpp
void buildScalarUI();
```
Build scalar ui.

```cpp
virtual void buildScalarOptionsUI();
```
Build scalar options ui.

```cpp
std::vector<std::string> addScalarRules(std::vector<std::string> rules);
```
Add scalar rules.

```cpp
void setScalarUniforms(render::ShaderProgram& p);
```
Set scalar uniforms.

```cpp
template <class V> void updateData(const V& newValues);
```
Update data.

```cpp
QuantityT& quantity;
```
Member quantity.

```cpp
render::ManagedBuffer<float> values;
```
Member values.

```cpp
QuantityT* setColorMap(std::string val);
```
Set color map.

```cpp
std::string getColorMap();
```
Get color map.

```cpp
QuantityT* setMapRange(std::pair<double, double> val);
```
Set map range.

```cpp
std::pair<double, double> getMapRange();
```
Get map range.

```cpp
QuantityT* resetMapRange();
```
Reset map range.

```cpp
std::pair<double, double> getDataRange();
```
Get data range.

```cpp
QuantityT* setIsolinesEnabled(bool newEnabled);
```
Set isolines enabled.

```cpp
bool getIsolinesEnabled();
```
Get isolines enabled.

```cpp
QuantityT* setIsolineStyle(IsolineStyle val);
```
Set isoline style.

```cpp
IsolineStyle getIsolineStyle();
```
Get isoline style.

```cpp
QuantityT* setIsolinePeriod(double size, bool isRelative);
```
Set isoline period.

```cpp
double getIsolinePeriod();
```
Get isoline period.

```cpp
QuantityT* setIsolineDarkness(double val);
```
Set isoline darkness.

```cpp
double getIsolineDarkness();
```
Get isoline darkness.

```cpp
QuantityT* setIsolineContourThickness(double val);
```
Set isoline contour thickness.

```cpp
double getIsolineContourThickness();
```
Get isoline contour thickness.

```cpp
QuantityT* setIsolineWidth(double size, bool isRelative);
```
Set isoline width.

```cpp
double getIsolineWidth();
```
Get isoline width.

```cpp
std::vector<float> valuesData;
```
Member values data.

```cpp
const DataType dataType;
```
Member data type.

```cpp
std::pair<double, double> dataRange;
```
Member double.

```cpp
std::pair<double, double> dataRange;
```
Member data range.

```cpp
PersistentValue<float> vizRangeMin;
```
Member viz range min.

```cpp
PersistentValue<float> vizRangeMax;
```
Member viz range max.

```cpp
Histogram hist;
```
Member hist.

```cpp
PersistentValue<std::string> cMap;
```
Member c map.

```cpp
PersistentValue<bool> isolinesEnabled;
```
Member isolines enabled.

```cpp
PersistentValue<IsolineStyle> isolineStyle;
```
Member isoline style.

```cpp
PersistentValue<ScaledValue<float>> isolinePeriod;
```
Member isoline period.

```cpp
PersistentValue<float> isolineDarkness;
```
Member isoline darkness.

```cpp
PersistentValue<float> isolineContourThickness;
```
Member isoline contour thickness.

## scalar_render_image_quantity.h

```cpp
class ScalarRenderImageQuantity : public RenderImageQuantityBase, public ScalarQuantity<ScalarRenderImageQuantity> {
```
Class ScalarRenderImageQuantity.

```cpp
ScalarRenderImageQuantity(Structure& parent_, std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& normalData, const std::vector<float>& scalarData, ImageOrigin imageOrigin, DataType dataType);
```
Scalar render image quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
template <typename T1, typename T2, typename T3> void updateBuffers(const T1& depthData, const T2& normalData, const T3& scalarData);
```
Update buffers.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
void prepare();
```
Prepare.

```cpp
template <typename T1, typename T2, typename T3> void ScalarRenderImageQuantity::updateBuffers(const T1& depthData, const T2& normalData, const T3& scalarData) { validateSize(depthData, dimX * dimY, "scalar render image depth data " + name);
```
Update buffers.

```cpp
validateSize(normalData, {dimX * dimY, 0}, "scalar render image normal data " + name);
```
Validate size.

```cpp
validateSize(scalarData, dimX * dimY, "scalar render image color data " + name);
```
Validate size.

```cpp
std::vector<float> standardDepth(standardizeArray<float>(depthData));
```
Standard depth.

```cpp
std::vector<glm::vec3> standardNormal(standardizeVectorArray<glm::vec3, 3>(normalData));
```
Standard normal.

```cpp
std::vector<float> standardScalar(standardizeArray<float>(scalarData));
```
Standard scalar.

```cpp
values.markHostBufferUpdated();
```
Mark host buffer updated.

```cpp
updateBaseBuffers(standardDepth, standardNormal);
```
Update base buffers.

## scaled_value.h

```cpp
template <typename T> class ScaledValue {
```
Class ScaledValue.

```cpp
ScaledValue() : relativeFlag(true), value() {} ScaledValue(T value_, bool relativeFlag_) : relativeFlag(relativeFlag_), value(value_) {} template <typename U> ScaledValue(const ScaledValue<U>& otherValue) : relativeFlag(otherValue.relativeFlag), value(otherValue.value) {} static ScaledValue<T> relative(T value_) { return ScaledValue<T>(value_, true); }
```
Scaled value.

```cpp
static ScaledValue<T> absolute(T value_) { return ScaledValue<T>(value_, false); }
```
Absolute.

```cpp
T asAbsolute() const { return relativeFlag ? value * state::lengthScale : value; }
```
As absolute.

```cpp
T* getValuePtr() { return &value; }
```
Get value ptr.

```cpp
bool isRelative() const { return relativeFlag; }
```
Return whether relative.

```cpp
void set(T value_, bool relativeFlag_ = true) { value = value_;
```
Set .

```cpp
relativeFlag = relativeFlag_;
```
Member relative flag.

```cpp
} ScaledValue(const T& relativeValue) : relativeFlag(true), value(relativeValue) {} bool operator==(const ScaledValue<T>& rhs) const { return (value == rhs.value) && (relativeFlag == rhs.relativeFlag);
```
Scaled value.

```cpp
} bool operator!=(const ScaledValue<T>& rhs) const { return !operator==(rhs); }
```
Function.

```cpp
template <typename> friend class ScaledValue;
```
Member scaled value.

```cpp
bool relativeFlag;
```
Member relative flag.

```cpp
T value;
```
Member value.

```cpp
template <typename T> ScaledValue<T> absoluteValue(T val) { return ScaledValue<T>::absolute(val);
```
Absolute value.

```cpp
template <typename T> ScaledValue<T> relativeValue(T val) { return ScaledValue<T>::relative(val);
```
Relative value.

## screenshot.h

```cpp
struct ScreenshotOptions {
```
Struct ScreenshotOptions.

```cpp
bool transparentBackground = true;
```
Member true.

```cpp
bool includeUI = false;
```
Member false.

```cpp
void screenshot(const ScreenshotOptions& options = {}); // automatic file names like `screenshot_000000.png` void screenshot(std::string filename, const ScreenshotOptions& options = {});
```
Screenshot.

```cpp
std::vector<unsigned char> screenshotToBuffer(const ScreenshotOptions& options = {});
```
Screenshot to buffer.

```cpp
void screenshot(bool transparentBG); // automatic file names like `screenshot_000000.png` void screenshot(std::string filename, bool transparentBG = true);
```
Screenshot.

```cpp
void screenshot(const char* filename); // this is needed because annoyingly overload resolution prefers the bool version void saveImage(std::string name, unsigned char* buffer, int w, int h, int channels); // helper void resetScreenshotIndex();
```
Screenshot.

```cpp
std::vector<unsigned char> screenshotToBuffer(bool transparentBG);
```
Screenshot to buffer.

## simple_triangle_mesh.h

```cpp
class SimpleTriangleMesh;
```
Forward declare simple triangle mesh

```cpp
struct SimpleTriangleMeshPickResult {
```
Forward declare quantity types template <> // Specialize the quantity type struct QuantityTypeHelper<SimpleTriangleMesh> { typedef SimpleTriangleMeshQuantity type; };

```cpp
class SimpleTriangleMesh : public QuantityStructure<SimpleTriangleMesh> {
```
Class SimpleTriangleMesh.

```cpp
SimpleTriangleMesh(std::string name, std::vector<glm::vec3> vertices, std::vector<glm::uvec3> faces);
```
Simple triangle mesh.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void buildPickUI(const PickResult& result) override;
```
Build pick ui.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
render::ManagedBuffer<glm::vec3> vertices;
```
Member vertices.

```cpp
render::ManagedBuffer<glm::uvec3> faces;
```
Member faces.

```cpp
template <class V> void updateVertices(const V& newPositions);
```
Update vertices.

```cpp
template <class V, class F> void update(const V& newVertices, const F& newFaces);
```
Update.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
SimpleTriangleMeshPickResult interpretPickResult(const PickResult& result);
```
Interpret pick result.

```cpp
SimpleTriangleMesh* setSurfaceColor(glm::vec3 newVal);
```
Set surface color.

```cpp
glm::vec3 getSurfaceColor();
```
Get surface color.

```cpp
SimpleTriangleMesh* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
SimpleTriangleMesh* setBackFaceColor(glm::vec3 val);
```
Set back face color.

```cpp
glm::vec3 getBackFaceColor();
```
Get back face color.

```cpp
SimpleTriangleMesh* setBackFacePolicy(BackFacePolicy newPolicy);
```
Set back face policy.

```cpp
BackFacePolicy getBackFacePolicy();
```
Get back face policy.

```cpp
void setSimpleTriangleMeshUniforms(render::ShaderProgram& p, bool withSurfaceShade = true);
```
Set simple triangle mesh uniforms.

```cpp
void setSimpleTriangleMeshProgramGeometryAttributes(render::ShaderProgram& p);
```
Set simple triangle mesh program geometry attributes.

```cpp
std::vector<std::string> addSimpleTriangleMeshRules(std::vector<std::string> initRules, bool withSurfaceShade = true);
```
Add simple triangle mesh rules.

```cpp
std::vector<glm::vec3> verticesData;
```
Member vertices data.

```cpp
std::vector<glm::uvec3> facesData;
```
Member faces data.

```cpp
PersistentValue<glm::vec3> surfaceColor;
```
Member surface color.

```cpp
PersistentValue<std::string> material;
```
Member material.

```cpp
PersistentValue<BackFacePolicy> backFacePolicy;
```
Member back face policy.

```cpp
PersistentValue<glm::vec3> backFaceColor;
```
Member back face color.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
std::shared_ptr<render::ShaderProgram> pickProgram;
```
Member pick program.

```cpp
void ensureRenderProgramPrepared();
```
Ensure render program prepared.

```cpp
void ensurePickProgramPrepared();
```
Ensure pick program prepared.

```cpp
void setPickUniforms(render::ShaderProgram& p);
```
Set pick uniforms.

```cpp
size_t pickStart;
```
Member pick start.

```cpp
glm::vec3 pickColor;
```
Member pick color.

```cpp
template <class V, class F> SimpleTriangleMesh* registerSimpleTriangleMesh(std::string name, const V& vertices, const F& faces);
```
Register simple triangle mesh.

```cpp
inline SimpleTriangleMesh* getSimpleTriangleMesh(std::string name = "");
```
Get simple triangle mesh.

```cpp
inline bool hasSimpleTriangleMesh(std::string name = "");
```
Return whether it has simple triangle mesh.

```cpp
inline void removeSimpleTriangleMesh(std::string name = "", bool errorIfAbsent = false);
```
Remove simple triangle mesh.

## slice_plane.h

```cpp
class SlicePlane {
```
Class SlicePlane.

```cpp
SlicePlane(std::string name);
```
Slice plane.

```cpp
~SlicePlane();
```
Slice plane.

```cpp
SlicePlane(const SlicePlane&) = delete;
```
Slice plane.

```cpp
SlicePlane& operator=(const SlicePlane&) = delete;
```
Function.

```cpp
void buildGUI();
```
Build gui.

```cpp
void draw();
```
Draw.

```cpp
void drawGeometry();
```
Draw geometry.

```cpp
void resetVolumeSliceProgram();
```
Reset volume slice program.

```cpp
void ensureVolumeInspectValid();
```
Ensure volume inspect valid.

```cpp
void setSceneObjectUniforms(render::ShaderProgram& p, bool alwaysPass = false);
```
Set scene object uniforms.

```cpp
void setSliceGeomUniforms(render::ShaderProgram& p);
```
Set slice geom uniforms.

```cpp
const std::string name;
```
Member name.

```cpp
const std::string postfix;
```
Member postfix.

```cpp
std::string uniquePrefix();
```
Unique prefix.

```cpp
void setPose(glm::vec3 planePosition, glm::vec3 planeNormal);
```
Set pose.

```cpp
bool getActive();
```
Get active.

```cpp
void setActive(bool newVal);
```
Set active.

```cpp
bool getDrawPlane();
```
Get draw plane.

```cpp
void setDrawPlane(bool newVal);
```
Set draw plane.

```cpp
bool getDrawWidget();
```
Get draw widget.

```cpp
void setDrawWidget(bool newVal);
```
Set draw widget.

```cpp
glm::mat4 getTransform();
```
Get transform.

```cpp
void setTransform(glm::mat4 newTransform);
```
Set transform.

```cpp
void setColor(glm::vec3 newVal);
```
Set color.

```cpp
glm::vec3 getColor();
```
Get color.

```cpp
void setGridLineColor(glm::vec3 newVal);
```
Set grid line color.

```cpp
glm::vec3 getGridLineColor();
```
Get grid line color.

```cpp
void setTransparency(double newVal);
```
Set transparency.

```cpp
double getTransparency();
```
Get transparency.

```cpp
void setVolumeMeshToInspect(std::string meshName);
```
Set volume mesh to inspect.

```cpp
std::string getVolumeMeshToInspect();
```
Get volume mesh to inspect.

```cpp
PersistentValue<bool> active;
```
Member active.

```cpp
PersistentValue<bool> drawPlane;
```
Member draw plane.

```cpp
PersistentValue<bool> drawWidget;
```
Member draw widget.

```cpp
PersistentValue<glm::mat4> objectTransform;
```
Member object transform.

```cpp
PersistentValue<glm::vec3> color;
```
Member color.

```cpp
PersistentValue<glm::vec3> gridLineColor;
```
Member grid line color.

```cpp
PersistentValue<float> transparency;
```
Member transparency.

```cpp
bool shouldInspectMesh;
```
Member should inspect mesh.

```cpp
std::string inspectedMeshName;
```
Member inspected mesh name.

```cpp
std::shared_ptr<render::ShaderProgram> volumeInspectProgram;
```
Member volume inspect program.

```cpp
TransformationGizmo transformGizmo;
```
Member transform gizmo.

```cpp
std::array<std::vector<uint32_t>, 4> sliceBufferDataArr;
```
Member slice buffer data arr.

```cpp
std::array<render::ManagedBuffer<uint32_t>, 4> sliceBufferArr;
```
Member slice buffer arr.

```cpp
std::shared_ptr<render::ShaderProgram> planeProgram;
```
Member plane program.

```cpp
void setSliceAttributes(render::ShaderProgram& p);
```
Set slice attributes.

```cpp
void createVolumeSliceProgram();
```
Create volume slice program.

```cpp
void prepare();
```
Prepare.

```cpp
glm::vec3 getCenter();
```
Get center.

```cpp
glm::vec3 getNormal();
```
Get normal.

```cpp
void updateWidgetEnabled();
```
Update widget enabled.

```cpp
SlicePlane* addSceneSlicePlane(bool initiallyVisible = false);
```
Add scene slice plane.

```cpp
void removeLastSceneSlicePlane();
```
Remove last scene slice plane.

```cpp
void removeAllSlicePlanes();
```
Remove all slice planes.

```cpp
void buildSlicePlaneGUI();
```
Build slice plane gui.

## standardize_data_array.h

```cpp
struct PreferenceT : PreferenceT<N - 1> {};
```
Struct PreferenceT.

```cpp
struct PreferenceT<0> {};
```
Struct PreferenceT.

```cpp
struct WillBeFalseT : std::false_type {};
```
Struct WillBeFalseT.

```cpp
template <typename T> template <typename T> struct InnerType {
```
Struct InnerType.

```cpp
inline void adaptorF_custom_size(void* dont_use) { // dummy function } // Highest priority: any user defined function template <class T, /* condition: user function exists and returns something that can be cast to size_t */ typename C1 = typename std::enable_if< std::is_same<decltype((size_t)adaptorF_custom_size(std::declval<T>())), size_t>::value>::type> size_t adaptorF_sizeImpl(PreferenceT<4>, const T& inputData) { return adaptorF_custom_size(inputData);
```
Adaptor f custom size.

```cpp
template <class T, /* condition: has .rows() method which returns something that can be cast to size_t */ typename C1 = typename std::enable_if<std::is_same<decltype((size_t)(std::declval<T>()).rows()), size_t>::value>::type> size_t adaptorF_sizeImpl(PreferenceT<3>, const T& inputData) { return inputData.rows();
```
Rows.

```cpp
template <class T, /* condition: has .size() method which returns something that can be cast to size_t */ typename C1 = typename std::enable_if< std::is_same<decltype((size_t)(std::declval<T>()).size()), size_t>::value>::type> size_t adaptorF_sizeImpl(PreferenceT<2>, const T& inputData) { return inputData.size();
```
Size.

```cpp
template <class T, /* condition: std::get<1>() returns something that can be cast to size_t */ typename C1 = typename std::enable_if< std::is_same<decltype((size_t)std::get<1>(std::declval<T>())), size_t>::value>::type> size_t adaptorF_sizeImpl(PreferenceT<1>, const T& inputData) { return std::get<1>(inputData);
```
Decltype.

```cpp
template <class T, class S> size_t adaptorF_sizeImpl(PreferenceT<0>, const T& inputData) { static_assert(WillBeFalseT<T>::value, "could not resolve valid adaptor for size of array-like data");
```
Adaptor f size impl.

```cpp
template <class T> size_t adaptorF_size(const T& inputData) { return adaptorF_sizeImpl(PreferenceT<4>{}, inputData);
```
Adaptor f size.

```cpp
inline void adaptorF_custom_convertToStdVector(void* dont_use) { // dummy function } // Highest priority: user-specified function template <class T, class S, /* condition: user defined function exists and returns something that can be bracket-indexed to get an S */ typename C1 = typename std::enable_if< std::is_same<decltype((S)adaptorF_custom_convertToStdVector(std::declval<T>())[0]), S>::value>::type> void adaptorF_convertToStdVectorImpl(PreferenceT<5>, const T& inputData, std::vector<S>& out) { auto userVec = adaptorF_custom_convertToStdVector(inputData);
```
Adaptor f custom convert to std vector.

```cpp
out.resize(userVec.size());
```
Resize.

```cpp
for (size_t i = 0; i < out.size(); i++) { out[i] = userVec[i];
```
For.

```cpp
template <class T, class S, typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>())[(size_t)0]), S>::value>::type> void adaptorF_convertToStdVectorImpl(PreferenceT<4>, const T& inputData, std::vector<S>& dataOut) { size_t dataSize = adaptorF_size(inputData);
```
Decltype.

```cpp
dataOut.resize(dataSize);
```
Resize.

```cpp
for (size_t i = 0; i < dataSize; i++) { dataOut[i] = inputData[i];
```
For.

```cpp
template <class T, class S, /* condition: input can be called (aka parenthesis-indexed) to get an S */ typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>())((size_t)0)), S>::value>::type> void adaptorF_convertToStdVectorImpl(PreferenceT<3>, const T& inputData, std::vector<S>& dataOut) { size_t dataSize = adaptorF_size(inputData);
```
Called.

```cpp
dataOut.resize(dataSize);
```
Resize.

```cpp
for (size_t i = 0; i < dataSize; i++) { dataOut[i] = inputData(i);
```
For.

```cpp
template <class T, class S, /* condition: input has a begin() and end() function, both of which can be dereferenced to get an S */ typename C1 = typename std::enable_if<std::is_same<decltype((S)*std::begin(std::declval<T>())), S>::value && std::is_same<decltype((S)*std::end(std::declval<T>())), S>::value>::type> void adaptorF_convertToStdVectorImpl(PreferenceT<2>, const T& inputData, std::vector<S>& dataOut) { size_t dataSize = adaptorF_size(inputData);
```
Begin.

```cpp
dataOut.resize(dataSize);
```
Resize.

```cpp
for (auto v : inputData) { dataOut[i] = v;
```
For.

```cpp
template <class T, class S, typename C_DATA = decltype(static_cast<S>(*std::get<0>(std::declval<T>()))), /* condition: second entry of input is castable to an index type */ typename C_COUNT = decltype(static_cast<size_t>(std::get<1>(std::declval<T>()))) > void adaptorF_convertToStdVectorImpl(PreferenceT<1>, const T& inputData, std::vector<S>& dataOut) { size_t dataSize = adaptorF_size(inputData);
```
Decltype.

```cpp
dataOut.resize(dataSize);
```
Resize.

```cpp
for (size_t i = 0; i < dataSize; i++) { dataOut[i] = dataPtr[i];
```
For.

```cpp
template <class T, class S> void adaptorF_convertToStdVectorImpl(PreferenceT<0>, const T& inputData, std::vector<S>& dataOut) { static_assert(WillBeFalseT<T>::value, "could not resolve valid adaptor for accessing array-like data");
```
Adaptor f convert to std vector impl.

```cpp
template <class S, class T> void adaptorF_convertToStdVector(const T& inputData, std::vector<S>& dataOut) { adaptorF_convertToStdVectorImpl<T, S>(PreferenceT<5>{}, inputData, dataOut);
```
Adaptor f convert to std vector.

```cpp
inline void adaptorF_custom_accessVector2Value(void* dont_use) { // dummy function } // Highest priority: any user defined function template <unsigned int I, class T, class S, /* condition: user function exists and retuns something that can be cast to an S */ typename C1 = typename std::enable_if< std::is_same<decltype((S)adaptorF_custom_accessVector2Value(std::declval<T>(), 0)), S>::value>::type> S adaptorF_accessVector2ValueImpl(PreferenceT<5>, const T& inputVec) { static_assert(I < 2, "bad vector2 access");
```
Adaptor f custom access vector 2 value.

```cpp
return adaptorF_custom_accessVector2Value(inputVec, I);
```
Adaptor f custom access vector 2 value.

```cpp
template <unsigned int I, class T, class S, typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>())[0]), S>::value>::type> S adaptorF_accessVector2ValueImpl(PreferenceT<4>, const T& inputVec) { static_assert(I < 2, "bad vector2 access");
```
Decltype.

```cpp
return (S)inputVec[I];
```
Return.

```cpp
template <unsigned int I, class T, class S, typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>()).x), S>::value && std::is_same<decltype((S)(std::declval<T>()).y), S>::value>::type> S adaptorF_accessVector2ValueImpl(PreferenceT<3>, const T& inputVec) { static_assert(I < 2, "bad vector2 access");
```
Decltype.

```cpp
if (I == 0) { return (S)inputVec.x;
```
If.

```cpp
return (S)inputVec.y;
```
Return.

```cpp
template <unsigned int I, class T, class S, typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>()).u), S>::value && std::is_same<decltype((S)(std::declval<T>()).v), S>::value>::type> S adaptorF_accessVector2ValueImpl(PreferenceT<2>, const T& inputVec) { static_assert(I < 2, "bad vector2 access");
```
Decltype.

```cpp
if (I == 0) { return (S)inputVec.u;
```
If.

```cpp
return (S)inputVec.v;
```
Return.

```cpp
template <unsigned int I, class T, class S, /* condition: input has .real() and .imag() member functions which give something that can be cast to an S */ typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>()).real()), S>::value && std::is_same<decltype((S)(std::declval<T>()).imag()), S>::value>::type> S adaptorF_accessVector2ValueImpl(PreferenceT<1>, const T& inputVec) { static_assert(I < 2, "bad vector2 access");
```
Real.

```cpp
if (I == 0) { return (S)inputVec.real();
```
If.

```cpp
return (S)inputVec.imag();
```
Return.

```cpp
template <unsigned int I, class T, class S> S adaptorF_accessVector2ValueImpl(PreferenceT<0>, const T& inputVec) { static_assert(WillBeFalseT<T>::value, "could not resolve valid accessor for 2D vector-like value");
```
Adaptor f access vector 2 value impl.

```cpp
return S();
```
S.

```cpp
template <class S, unsigned int I, class T, class C1 = typename std::enable_if< (I < 2) >::type> class C1 = typename std::enable_if< (I < 2) >::type> S adaptorF_accessVector2Value(const T& inVal) {
```
Class C1.

```cpp
return adaptorF_accessVector2ValueImpl<I, T, S>(PreferenceT<5>{}, inVal);
```
Function.

```cpp
inline void adaptorF_custom_accessVector3Value(void* dont_use) { // dummy function } // Highest priority: any user defined function template <unsigned int I, class T, class S, /* condition: user function exists and returns something that can be cast to S */ typename C1 = typename std::enable_if< std::is_same<decltype((S)adaptorF_custom_accessVector3Value(std::declval<T>(), 0)), S>::value>::type> S adaptorF_accessVector3ValueImpl(PreferenceT<3>, const T& inputVec) { static_assert(I < 3, "bad vector3 access");
```
Adaptor f custom access vector 3 value.

```cpp
return adaptorF_custom_accessVector3Value(inputVec, I);
```
Adaptor f custom access vector 3 value.

```cpp
template <unsigned int I, class T, class S, typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>())[0]), S>::value>::type> S adaptorF_accessVector3ValueImpl(PreferenceT<2>, const T& inputVec) { static_assert(I < 3, "bad vector3 access");
```
Decltype.

```cpp
return (S)inputVec[I];
```
Return.

```cpp
template <unsigned int I, class T, class S, typename C1 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>()).x), S>::value && std::is_same<decltype((S)(std::declval<T>()).y), S>::value && std::is_same<decltype((S)(std::declval<T>()).z), S>::value>::type> S adaptorF_accessVector3ValueImpl(PreferenceT<1>, const T& inputVec) { static_assert(I < 3, "bad vector3 access");
```
Decltype.

```cpp
if (I == 0) { return (S)inputVec.x;
```
If.

```cpp
} else if (I == 1) { return (S)inputVec.y;
```
If.

```cpp
return (S)inputVec.z;
```
Return.

```cpp
template <unsigned int I, class T, class S> S adaptorF_accessVector3ValueImpl(PreferenceT<0>, const T& inputVec) { static_assert(WillBeFalseT<T>::value, "could not resolve valid accessor for 3D vector-like value");
```
Adaptor f access vector 3 value impl.

```cpp
return S();
```
S.

```cpp
template <class S, unsigned int I, class T, class C1 = typename std::enable_if< (I < 3) >::type> class C1 = typename std::enable_if< (I < 3) >::type> S adaptorF_accessVector3Value(const T& inVal) {
```
Class C1.

```cpp
return adaptorF_accessVector3ValueImpl<I, T, S>(PreferenceT<3>{}, inVal);
```
Function.

```cpp
inline void adaptorF_custom_convertArrayOfVectorToStdVector(void* dont_use) { // dummy function } template < class O, unsigned int D, class T, /* condition: user function exists and returns something that can be bracket-indexed to get an S */ typename C1 = typename std::enable_if<std::is_same< decltype((typename InnerType<O>::type)(adaptorF_custom_convertArrayOfVectorToStdVector(std::declval<T>()))[0][0]), typename InnerType<O>::type>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<9>, const T& inputData) { // should be std::vector<std::array<SCALAR,D>> auto userArr = adaptorF_custom_convertArrayOfVectorToStdVector(inputData);
```
Adaptor f custom convert array of vector to std vector.

```cpp
size_t dataSize = userArr.size();
```
Size.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (size_t i = 0; i < dataSize; i++) { for (size_t j = 0; j < D; j++) { dataOut[i][j] = userArr[i][j];
```
For.

```cpp
template <class O, unsigned int D, class T, typename C1 = typename std::enable_if<std::is_same< decltype((typename InnerType<O>::type)(std::declval<T>())((size_t)0, (size_t)0)), typename InnerType<O>::type>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<8>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Decltype.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (size_t i = 0; i < dataSize; i++) { for (size_t j = 0; j < D; j++) { dataOut[i][j] = inputData(i, j);
```
For.

```cpp
template <class O, unsigned int D, class T, typename C1 = typename std::enable_if<std::is_same< decltype((typename InnerType<O>::type)(std::declval<T>())[(size_t)0][(size_t)0]), typename InnerType<O>::type>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<7>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Decltype.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (size_t i = 0; i < dataSize; i++) { for (size_t j = 0; j < D; j++) { dataOut[i][j] = inputData[i][j];
```
For.

```cpp
template <class O, unsigned int D, class T, typename C_INNER = typename std::remove_reference<decltype((std::declval<T>())[(size_t)0])>::type, /* helper type: inner type of output O */ typename C_RES = typename InnerType<O>::type, /* helper type: scalar type that results from a vector3 access on C_INNER */ typename C_INNER_SCALAR = decltype(adaptorF_accessVector3Value<C_RES, 0>((std::declval<C_INNER>()))), /* condition: output dimension must be 3 */ typename C1 = typename std::enable_if<D == 3>::type, /* condition: the inner_scalar that comes from the vector3 unpack must match the requested inner type */ typename C2 = typename std::enable_if<std::is_same<C_INNER_SCALAR, C_RES>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<6>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Decltype.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (size_t i = 0; i < dataSize; i++) { dataOut[i][0] = adaptorF_accessVector3Value<C_RES, 0>(inputData[i]);
```
For.

```cpp
template <class O, unsigned int D, class T, typename C_INNER = typename std::remove_reference<decltype((std::declval<T>())[(size_t)0])>::type, /* helper type: inner type of output O */ typename C_RES = typename InnerType<O>::type, /* helper type: scalar type that results from a vector2 access on C_INNER */ typename C_INNER_SCALAR = decltype(adaptorF_accessVector2Value<C_RES, 0>((std::declval<C_INNER>()))), /* condition: output dimension must be 2 */ typename C1 = typename std::enable_if<D == 2>::type, /* condition: the inner_scalar that comes from the vector2 unpack must match the requested inner type */ typename C2 = typename std::enable_if<std::is_same<C_INNER_SCALAR, C_RES>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<5>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Decltype.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (size_t i = 0; i < dataSize; i++) { dataOut[i][0] = adaptorF_accessVector2Value<C_RES, 0>(inputData[i]);
```
For.

```cpp
template <class O, unsigned int D, class T, /* helper type: inner type that results from dereferencing begin() */ typename C_INNER = typename std::remove_reference<decltype(*(std::declval<T>()).begin())>::type, /* helper type: inner type that results from dereferencing end() */ typename C_INNER_END = typename std::remove_reference<decltype(*(std::declval<T>()).end())>::type, /* helper type: inner type of output O */ typename C_RES = typename InnerType<O>::type, /* helper type: scalar type that results from a vector3 access on C_INNER */ typename C_INNER_SCALAR = decltype(adaptorF_accessVector3Value<C_RES, 0>((std::declval<C_INNER>()))), /* condition: output dimension must be 3 */ typename C1 = typename std::enable_if<D == 3>::type, /* condition: the inner_scalar that comes from the vector3 unpack must match the requested inner type */ typename C2 = typename std::enable_if<std::is_same<C_INNER_SCALAR, C_RES>::value>::type, /* condition: the type that comes from begin() must match the one from end() */ typename C3 = typename std::enable_if<std::is_same<C_INNER, C_INNER_END>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<4>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Begin.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (auto v : inputData) { dataOut[i][0] = adaptorF_accessVector3Value<C_RES, 0>(v);
```
For.

```cpp
template <class O, unsigned int D, class T, /* helper type: inner type that results from dereferencing begin() */ typename C_INNER = typename std::remove_reference<decltype(*(std::declval<T>()).begin())>::type, /* helper type: inner type that results from dereferencing end() */ typename C_INNER_END = typename std::remove_reference<decltype(*(std::declval<T>()).end())>::type, /* helper type: inner type of output O */ typename C_RES = typename InnerType<O>::type, /* helper type: scalar type that results from a vector2 access on C_INNER */ typename C_INNER_SCALAR = decltype(adaptorF_accessVector2Value<C_RES, 0>((std::declval<C_INNER>()))), /* condition: output dimension must be 2 */ typename C1 = typename std::enable_if<D == 2>::type, /* condition: the inner_scalar that comes from the vector2 unpack must match the requested inner type */ typename C2 = typename std::enable_if<std::is_same<C_INNER_SCALAR, C_RES>::value>::type, /* condition: the type that comes from begin() must match the one from end() */ typename C3 = typename std::enable_if<std::is_same<C_INNER, C_INNER_END>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<3>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Begin.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (auto v : inputData) { dataOut[i][0] = adaptorF_accessVector2Value<C_RES, 0>(v);
```
For.

```cpp
template <class O, unsigned int D, class T, typename C_RES = typename InnerType<O>::type, /* condition: begin() and end() should return something bracket-indexable to yield the inner type of O */ typename C1 = typename std::enable_if<std::is_same<decltype((C_RES)(*std::begin(std::declval<T>()))[0]), C_RES>::value && std::is_same<decltype((C_RES)(*std::end(std::declval<T>()))[0]), C_RES>::value>::type> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<2>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Begin.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (auto v : inputData) { for (size_t j = 0; j < D; j++) { dataOut[i][j] = v[j];
```
For.

```cpp
template <class O, unsigned int D, class T, typename C_DATA = decltype(static_cast<typename InnerType<O>::type>(*std::get<0>(std::declval<T>()))), /* condition: second entry of input is castable to an index type */ typename C_COUNT = decltype(static_cast<size_t>(std::get<1>(std::declval<T>()))) > std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<1>, const T& inputData) { size_t dataSize = adaptorF_size(inputData);
```
Decltype.

```cpp
std::vector<O> dataOut(dataSize);
```
Data out.

```cpp
for (size_t i = 0; i < dataSize; i++) { for (size_t j = 0; j < D; j++) { dataOut[i][j] = dataPtr[D * i + j];
```
For.

```cpp
template <class O, unsigned int D, class T> std::vector<O> adaptorF_convertArrayOfVectorToStdVectorImpl(PreferenceT<0>, const T& inputData) { static_assert(WillBeFalseT<T>::value, "could not resolve valid adaptor for accessing array-of-vectors-like input data");
```
Adaptor f convert array of vector to std vector impl.

```cpp
template <class O, unsigned int D, class T> std::vector<O> adaptorF_convertArrayOfVectorToStdVector(const T& inputData) { return adaptorF_convertArrayOfVectorToStdVectorImpl<O, D, T>(PreferenceT<9>{}, inputData);
```
Adaptor f convert array of vector to std vector.

```cpp
inline void adaptorF_custom_convertNestedArrayToStdVector(void* dont_use) { // dummy function } // Highest priority: user-specified function template <class S, class I, class T, /* condition: user function must be return a tuple of vectors with the compatible type (techincally this just checks for bracket-indexible-thing */ typename C1 = typename std::enable_if<std::is_same<decltype((S)std::get<0>(adaptorF_custom_convertNestedArrayToStdVector(std::declval<T>()))[0]), S>::value>::type, typename C2 = typename std::enable_if<std::is_same<decltype((I)std::get<1>(adaptorF_custom_convertNestedArrayToStdVector(std::declval<T>()))[0]), I>::value>::type > std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVectorImpl(PreferenceT<6>, const T& inputData) { // should be std::tuple<std::vector<S>, std::vector<I>> auto userArrTuple = adaptorF_custom_convertNestedArrayToStdVector(inputData); auto userDataArr = std::get<0>(userArrTuple); auto userDataStartArr = std::get<1>(userArrTuple); // TODO: question for past Nick, why do we do this copy for every custom function? // create output tuples size_t dataSize = userDataArr.size(); size_t dataStartSize = userDataStartArr.size(); std::tuple<std::vector<S>, std::vector<I>> outTuple; std::vector<S>& dataOut = std::get<0>(outTuple); std::vector<I>& dataStartOut = std::get<1>(outTuple); dataOut.resize(dataSize); dataStartOut.resize(dataStartSize); // copy data over for (size_t i = 0; i < dataSize; i++) dataOut[i] = userDataArr[i]; for (size_t i = 0; i < dataStartSize; i++) dataStartOut[i] = userDataStartArr[i]; return outTuple; } // Next: any dense callable (parenthesis) access operator template <class S, class I, class T, /* condition: must have .rows() function which return something like size_t */ typename C1 = typename std::enable_if<std::is_same<decltype((size_t)(std::declval<T>()).rows()), size_t>::value>::type, /* condition: must have .cols() function which return something like size_t */ typename C2 = typename std::enable_if<std::is_same<decltype((size_t)(std::declval<T>()).cols()), size_t>::value>::type, /* condition: must have be able to call with two size_t arguments to get something that can be cast to S */ typename C3 = typename std::enable_if<std::is_same<decltype((S)(std::declval<T>())((size_t)0, (size_t)0)), S>::value>::type> std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVectorImpl(PreferenceT<5>, const T& inputData) { size_t outerSize = (size_t)inputData.rows(); size_t innerSize = (size_t)inputData.cols(); std::tuple<std::vector<S>, std::vector<I>> outTuple; std::vector<S>& dataOut = std::get<0>(outTuple); std::vector<I>& dataStartOut = std::get<1>(outTuple); dataOut.resize(outerSize*innerSize); dataStartOut.resize(outerSize+1); dataStartOut[0] = 0; for (size_t i = 0; i < outerSize; i++) { for (size_t j = 0; j < innerSize; j++) { dataOut[innerSize * i + j] = inputData(i, j); } dataStartOut[i+1] = innerSize * (i + 1); } return outTuple; } // Next: recusive unpacking with bracket template <class S, class I, class T, /* helper type: the result of bracket access on the outer type */ typename T_INNER = typename std::remove_reference<decltype((std::declval<T>())[0])>::type, /* condition: ensure that calling the inner array adaptor works */ typename C1 = decltype(adaptorF_convertToStdVector<S>(std::declval<T_INNER>(), std::declval<std::vector<S>&>())) > std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVectorImpl(PreferenceT<4>, const T& inputData) { size_t outerSize = adaptorF_size(inputData); std::tuple<std::vector<S>, std::vector<I>> outTuple; std::vector<S>& dataOut = std::get<0>(outTuple); std::vector<I>& dataStartOut = std::get<1>(outTuple); dataStartOut.resize(outerSize+1); std::vector<S> tempVec; for (size_t i = 0; i < outerSize; i++) { adaptorF_convertToStdVector<S>(inputData[i], tempVec); for(auto& s : tempVec) { dataOut.push_back(s); } dataStartOut[i+1] = dataOut.size(); } return outTuple; } // Next: recusive unpacking with paren template <class S, class I, class T, /* helper type: the result of paren access on the outer type */ typename T_INNER = typename std::remove_reference<decltype((std::declval<T>())(0))>::type, /* condition: ensure that calling the inner array adaptor works */ typename C1 = decltype(adaptorF_convertToStdVector<S>(std::declval<T_INNER>(), std::declval<std::vector<S>&>())) > std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVectorImpl(PreferenceT<3>, const T& inputData) { size_t outerSize = adaptorF_size(inputData); std::tuple<std::vector<S>, std::vector<I>> outTuple; std::vector<S>& dataOut = std::get<0>(outTuple); std::vector<I>& dataStartOut = std::get<1>(outTuple); dataStartOut.resize(outerSize+1); std::vector<S> tempVec; for (size_t i = 0; i < outerSize; i++) { adaptorF_convertToStdVector<S>(inputData(i), tempVec); for(auto& s : tempVec) { dataOut.push_back(s); } dataStartOut[i+1] = dataOut.size(); } return outTuple; } // Next: recusive unpacking with iterable template <class S, class I, class T, /* helper type: the result of dereferencing begin() on the outer type */ typename T_INNER = typename std::remove_reference<decltype(*(std::declval<T>()).begin())>::type, /* helper type: the result of dereferencing end() on the outer type */ typename T_INNER_END = typename std::remove_reference<decltype(*(std::declval<T>()).end())>::type, /* condition: ensure that calling the inner array adaptor works */ typename C1 = decltype(adaptorF_convertToStdVector<S>(std::declval<T_INNER>(), std::declval<std::vector<S>&>())), /* condition: T_INNER must match T_INNER_END */ typename C2 = typename std::enable_if< std::is_same<T_INNER, T_INNER_END>::value>::type > std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVectorImpl(PreferenceT<2>, const T& inputData) { size_t outerSize = adaptorF_size(inputData); std::tuple<std::vector<S>, std::vector<I>> outTuple; std::vector<S>& dataOut = std::get<0>(outTuple); std::vector<I>& dataStartOut = std::get<1>(outTuple); dataStartOut.resize(outerSize+1); std::vector<S> tempVec; size_t i = 0; for (const auto& n : inputData) { adaptorF_convertToStdVector<S>(n, tempVec); for(auto& s : tempVec) { dataOut.push_back(s); } dataStartOut[i+1] = dataOut.size(); i++; } return outTuple; } // Next: tuple {data_ptr, outer_size, inner_size} // This is LIMITED to rectangular data only // A Fx3 array would be passed as {ptr, F, 3} template <class S, class I, class T, /* condition: first entry of input can be dereferenced to get a type castable to the scalar type O */ typename C_DATA = decltype(static_cast<S>(*std::get<0>(std::declval<T>()))), /* condition: second & third entry of input is castable to an index type */ typename C_OUTER_COUNT = decltype(static_cast<size_t>(std::get<1>(std::declval<T>()))), typename C_INNER_COUNT = decltype(static_cast<size_t>(std::get<2>(std::declval<T>()))) > std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVectorImpl(PreferenceT<1>, const T& inputData) { auto* dataPtr = std::get<0>(inputData); size_t outerSize = static_cast<size_t>(std::get<1>(inputData)); size_t innerSize = static_cast<size_t>(std::get<2>(inputData)); std::tuple<std::vector<S>, std::vector<I>> outTuple; std::vector<S>& dataOut = std::get<0>(outTuple); std::vector<I>& dataStartOut = std::get<1>(outTuple); dataOut.resize(outerSize * innerSize); dataStartOut.resize(outerSize+1); dataStartOut[0] = 0; for (size_t i = 0; i < outerSize * innerSize; i++) { dataOut[i] = dataPtr[i]; } for (size_t i = 1; i <= outerSize; i++) { dataStartOut[i] = i * innerSize; } return outTuple; } // Fall-through case: no overload found :( // We use this to print a slightly less scary error message. #ifndef POLYSCOPE_NO_STANDARDIZE_FALLTHROUGH template <class S, class I, class T> std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVector(PreferenceT<0>, const T& inputData) { static_assert(WillBeFalseT<T>::value, "could not resolve valid adaptor for accessing nested-array-like input data"); return std::tuple<std::vector<S>, std::vector<I>>(); } #endif // General version, which will attempt to substitute in to the variants above template <class S, class I, class T> std::tuple<std::vector<S>, std::vector<I>> adaptorF_convertNestedArrayToStdVector(const T& inputData) { return adaptorF_convertNestedArrayToStdVectorImpl<S, I, T>(PreferenceT<6>{}, inputData); } // clang-format on // ================================================= // ============ standardize functions // ================================================= // These are utilitiy functions which use the adaptors above to do useful things, with nicer syntax. // Check that a data array has the expected size template <class T> void validateSize(const T& inputData, std::vector<size_t> expectedSizes, std::string errorName = "") { // No-op if no sizes given if (expectedSizes.size() == 0) { return; } size_t dataSize = adaptorF_size(inputData); // Simpler error if only one size if (expectedSizes.size() == 1) { if (dataSize != expectedSizes[0]) { exception("Size validation failed on data array [" + errorName + "]. Expected size " + std::to_string(expectedSizes[0]) + " but has size " + std::to_string(dataSize)); } } // General case else { // Return success if any sizes match for (size_t possibleSize : expectedSizes) { if (dataSize == possibleSize) { return; } } // Build a useful error message std::string sizesStr = "{"; for (size_t possibleSize : expectedSizes) { sizesStr += std::to_string(possibleSize) + ","; } sizesStr += "}"; exception("Size validation failed on data array [" + errorName + "]. Expected size in " + sizesStr + " but has size " + std::to_string(dataSize)); } } // Pass through to general version which takes a single expected size template <class T> void validateSize(const T& inputData, size_t expectedSize, std::string errorName = "") { validateSize<T>(inputData, std::vector<size_t>{expectedSize}, errorName); } // Convert a single fixed-size 2D vector // class O: output vector type to put the result in. Will be bracket-indexed. // (Polyscope pretty much always uses glm::vec2/3 or std::array<>) // class T: input array type (must be a 2D vector) template <class O, class T> O standardizeVector2D(const T& inputVec) { O out; out[0] = adaptorF_accessVector2Value<decltype(out[0]), 0, T>(inputVec); out[1] = adaptorF_accessVector2Value<decltype(out[0]), 1, T>(inputVec); return out; } // Convert a single fixed-size 2D vector // class O: output vector type to put the result in. Will be bracket-indexed. // (Polyscope pretty much always uses glm::vec2/3 or std::array<>) // class T: input array type (must be a 3D vector) template <class O, class T> O standardizeVector3D(const T& inputVec) { O out; out[0] = adaptorF_accessVector3Value<decltype(out[0]), 0, T>(inputVec); out[1] = adaptorF_accessVector3Value<decltype(out[0]), 1, T>(inputVec); out[2] = adaptorF_accessVector3Value<decltype(out[0]), 2, T>(inputVec); return out; } // Convert an array of scalar types // class D: scalar data type // class T: input array type template <class D, class T> std::vector<D> standardizeArray(const T& inputData) { std::vector<D> out; adaptorF_convertToStdVector<D, T>(inputData, out); return out; } // Convert an array of vector types // class O: output inner vector type to put the result in. Will be bracket-indexed. // (Polyscope pretty much always uses glm::vec2/3, std::vector<>, or std::array<>) // unsigned int D: dimension of inner vector type // class T: input array type template <class O, unsigned int D, class T> std::vector<O> standardizeVectorArray(const T& inputData) { return adaptorF_convertArrayOfVectorToStdVector<O, D, T>(inputData); } // Convert a nested array where the inner types have variable length. // class S: innermost scalar type for output // class T: input nested array type template <class S, class I, class T> std::tuple<std::vector<S>, std::vector<I>> standardizeNestedList(const T& inputData) { return adaptorF_convertNestedArrayToStdVector<S, I>(inputData); } } // namespace polyscope
```
Adaptor f custom convert nested array to std vector.

## structure.h

```cpp
class Group;
```
forward declarations

```cpp
class Structure : public render::ManagedBufferRegistry, public virtual WeakReferrable {
```
A 'structure' in Polyscope terms, is an object with which we can associate data in the UI, such as a point cloud, or a mesh.

```cpp
Structure(std::string name, std::string subtypeName);
```
Structure.

```cpp
virtual ~Structure() = 0;
```
Structure.

```cpp
virtual void draw() = 0;
```
Draw.

```cpp
virtual void drawDelayed() = 0;
```
Draw delayed.

```cpp
virtual void drawPick() = 0;
```
Draw pick.

```cpp
std::vector<std::string> addStructureRules(std::vector<std::string> initRules);
```
Add structure rules.

```cpp
virtual void buildUI();
```
Build ui.

```cpp
virtual void buildCustomUI() = 0;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI();
```
Build custom options ui.

```cpp
virtual void buildStructureOptionsUI();
```
Build structure options ui.

```cpp
virtual void buildQuantitiesUI();
```
Build quantities ui.

```cpp
virtual void buildSharedStructureUI();
```
Build shared structure ui.

```cpp
virtual void buildPickUI(const PickResult& result) = 0;
```
Build pick ui.

```cpp
const std::string name;
```
Member name.

```cpp
std::string uniquePrefix();
```
Unique prefix.

```cpp
std::string getName() { return name; };
```
Get name.

```cpp
std::tuple<glm::vec3, glm::vec3> boundingBox();
```
Bounding box.

```cpp
float lengthScale();
```
Length scale.

```cpp
virtual bool hasExtents();
```
Return whether it has extents.

```cpp
virtual std::string typeName() = 0;
```
Type name.

```cpp
glm::mat4 getModelView();
```
Get model view.

```cpp
void centerBoundingBox();
```
Center bounding box.

```cpp
void rescaleToUnit();
```
Rescale to unit.

```cpp
void resetTransform();
```
Reset transform.

```cpp
void setTransform(glm::mat4x4 transform);
```
Set transform.

```cpp
void setPosition(glm::vec3 vec);
```
Set position.

```cpp
void translate(glm::vec3 vec);
```
Translate.

```cpp
glm::mat4x4 getTransform();
```
Get transform.

```cpp
glm::vec3 getPosition();
```
Get position.

```cpp
void setStructureUniforms(render::ShaderProgram& p);
```
Set structure uniforms.

```cpp
bool wantsCullPosition();
```
Wants cull position.

```cpp
virtual void refresh();
```
Refresh.

```cpp
void remove();
```
Remove .

```cpp
virtual Structure* setEnabled(bool newEnabled);
```
Set enabled.

```cpp
bool isEnabled();
```
Return whether enabled.

```cpp
void enableIsolate();
```
Enable isolate.

```cpp
void setEnabledAllOfType(bool newEnabled);
```
Set enabled all of type.

```cpp
void addToGroup(std::string groupName);
```
Add to group.

```cpp
void addToGroup(Group& group);
```
Add to group.

```cpp
Structure* setTransparency(float newVal);
```
Set transparency.

```cpp
float getTransparency();
```
Get transparency.

```cpp
Structure* setCullWholeElements(bool newVal);
```
Set cull whole elements.

```cpp
bool getCullWholeElements();
```
Get cull whole elements.

```cpp
Structure* setIgnoreSlicePlane(std::string name, bool newValue);
```
Set ignore slice plane.

```cpp
bool getIgnoreSlicePlane(std::string name);
```
Get ignore slice plane.

```cpp
Structure* setTransformGizmoEnabled(bool newVal);
```
Set transform gizmo enabled.

```cpp
bool getTransformGizmoEnabled();
```
Get transform gizmo enabled.

```cpp
PersistentValue<bool> enabled;
```
Member enabled.

```cpp
PersistentValue<glm::mat4> objectTransform;
```
Member object transform.

```cpp
PersistentValue<float> transparency;
```
Member transparency.

```cpp
TransformationGizmo transformGizmo;
```
Member transform gizmo.

```cpp
PersistentValue<bool> cullWholeElements;
```
Member cull whole elements.

```cpp
PersistentValue<std::vector<std::string>> ignoredSlicePlaneNames;
```
Member ignored slice plane names.

```cpp
std::tuple<glm::vec3, glm::vec3> objectSpaceBoundingBox;
```
Member vec 3.

```cpp
std::tuple<glm::vec3, glm::vec3> objectSpaceBoundingBox;
```
Member object space bounding box.

```cpp
float objectSpaceLengthScale;
```
Member object space length scale.

```cpp
virtual void updateObjectSpaceBounds() = 0;
```
Update object space bounds.

```cpp
bool registerStructure(Structure* structure, bool replaceIfPresent = true);
```
Register structure.

```cpp
class Quantity;
```
Can also manage quantities forward declarations

```cpp
class QuantityS;
```
Class QuantityS.

```cpp
class FloatingQuantity;
```
Floating quantity things

```cpp
class ScalarImageQuantity;
```
Class ScalarImageQuantity.

```cpp
class ColorImageQuantity;
```
Class ColorImageQuantity.

```cpp
class DepthRenderImageQuantity;
```
Class DepthRenderImageQuantity.

```cpp
class ColorRenderImageQuantity;
```
Class ColorRenderImageQuantity.

```cpp
class ScalarRenderImageQuantity;
```
Class ScalarRenderImageQuantity.

```cpp
class RawColorRenderImageQuantity;
```
Class RawColorRenderImageQuantity.

```cpp
class RawColorAlphaRenderImageQuantity;
```
Class RawColorAlphaRenderImageQuantity.

```cpp
template <typename T> struct QuantityTypeHelper {
```
Struct QuantityTypeHelper.

```cpp
template <typename S> // template on the derived type class QuantityStructure : public Structure {
```
Class QuantityStructure.

```cpp
QuantityStructure(std::string name, std::string subtypeName);
```
Quantity structure.

```cpp
virtual ~QuantityStructure() = 0;
```
Quantity structure.

```cpp
virtual void buildQuantitiesUI() override;
```
Build quantities ui.

```cpp
virtual void buildStructureOptionsUI() override;
```
Build structure options ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
void addQuantity(QuantityType* q, bool allowReplacement = true);
```
Add quantity.

```cpp
void addQuantity(FloatingQuantity* q, bool allowReplacement = true);
```
Add quantity.

```cpp
QuantityType* getQuantity(std::string name);
```
Get quantity.

```cpp
FloatingQuantity* getFloatingQuantity(std::string name);
```
Get floating quantity.

```cpp
void checkForQuantityWithNameAndDeleteOrError(std::string name, bool allowReplacement = true);
```
Check for quantity with name and delete or error.

```cpp
void removeQuantity(std::string name, bool errorIfAbsent = false);
```
Remove quantity.

```cpp
void removeAllQuantities();
```
Remove all quantities.

```cpp
void setDominantQuantity(QuantityS<S>* q);
```
Set dominant quantity.

```cpp
void clearDominantQuantity();
```
Clear dominant quantity.

```cpp
void setAllQuantitiesEnabled(bool newEnabled);
```
Set all quantities enabled.

```cpp
std::map<std::string, std::unique_ptr<QuantityType>> quantities;
```
Member string.

```cpp
std::map<std::string, std::unique_ptr<QuantityType>> quantities;
```
Member quantities.

```cpp
QuantityS<S>* dominantQuantity = nullptr;
```
Member nullptr.

```cpp
std::map<std::string, std::unique_ptr<FloatingQuantity>> floatingQuantities;
```
Member string.

```cpp
std::map<std::string, std::unique_ptr<FloatingQuantity>> floatingQuantities;
```
Member floating quantities.

```cpp
template <class T> ScalarImageQuantity* addScalarImageQuantity(std::string name, size_t dimX, size_t dimY, const T& values, ImageOrigin imageOrigin = ImageOrigin::UpperLeft, DataType type = DataType::STANDARD);
```
Add scalar image quantity.

```cpp
template <class T> ColorImageQuantity* addColorImageQuantity(std::string name, size_t dimX, size_t dimY, const T& values_rgb, ImageOrigin imageOrigin = ImageOrigin::UpperLeft);
```
Add color image quantity.

```cpp
template <class T> ColorImageQuantity* addColorAlphaImageQuantity(std::string name, size_t dimX, size_t dimY, const T& values_rgba, ImageOrigin imageOrigin = ImageOrigin::UpperLeft);
```
Add color alpha image quantity.

```cpp
template <class T1, class T2> DepthRenderImageQuantity* addDepthRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& normalData, ImageOrigin imageOrigin = ImageOrigin::UpperLeft);
```
Add depth render image quantity.

```cpp
template <class T1, class T2, class T3> ColorRenderImageQuantity* addColorRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& normalData, const T3& colorData, ImageOrigin imageOrigin = ImageOrigin::UpperLeft);
```
Add color render image quantity.

```cpp
template <class T1, class T2, class T3> ScalarRenderImageQuantity* addScalarRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& normalData, const T3& scalarData, ImageOrigin imageOrigin = ImageOrigin::UpperLeft, DataType type = DataType::STANDARD);
```
Add scalar render image quantity.

```cpp
template <class T1, class T2> RawColorRenderImageQuantity* addRawColorRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& colorData, ImageOrigin imageOrigin = ImageOrigin::UpperLeft);
```
Add raw color render image quantity.

```cpp
template <class T1, class T2> RawColorAlphaRenderImageQuantity* addRawColorAlphaRenderImageQuantity(std::string name, size_t dimX, size_t dimY, const T1& depthData, const T2& colorData, ImageOrigin imageOrigin = ImageOrigin::UpperLeft);
```
Add raw color alpha render image quantity.

```cpp
ScalarImageQuantity* addScalarImageQuantityImpl(std::string name, size_t dimX, size_t dimY, const std::vector<float>& values, ImageOrigin imageOrigin, DataType type);
```
Add scalar image quantity impl.

```cpp
ColorImageQuantity* addColorImageQuantityImpl(std::string name, size_t dimX, size_t dimY, const std::vector<glm::vec4>& values, ImageOrigin imageOrigin);
```
Add color image quantity impl.

```cpp
DepthRenderImageQuantity* addDepthRenderImageQuantityImpl(std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& normalData, ImageOrigin imageOrigin);
```
Add depth render image quantity impl.

```cpp
ColorRenderImageQuantity* addColorRenderImageQuantityImpl(std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& normalData, const std::vector<glm::vec3>& colorData, ImageOrigin imageOrigin);
```
Add color render image quantity impl.

```cpp
ScalarRenderImageQuantity* addScalarRenderImageQuantityImpl(std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& normalData, const std::vector<float>& scalarData, ImageOrigin imageOrigin, DataType type);
```
Add scalar render image quantity impl.

```cpp
RawColorRenderImageQuantity* addRawColorRenderImageQuantityImpl(std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec3>& colorData, ImageOrigin imageOrigin);
```
Add raw color render image quantity impl.

```cpp
RawColorAlphaRenderImageQuantity* addRawColorAlphaRenderImageQuantityImpl(std::string name, size_t dimX, size_t dimY, const std::vector<float>& depthData, const std::vector<glm::vec4>& colorData, ImageOrigin imageOrigin);
```
Add raw color alpha render image quantity impl.

## surface_color_quantity.h

```cpp
class SurfaceMeshQuantity;
```
forward declarations

```cpp
class SurfaceMesh;
```
Class SurfaceMesh.

```cpp
class SurfaceParameterizationQuantity;
```
Class SurfaceParameterizationQuantity.

```cpp
class SurfaceColorQuantity : public SurfaceMeshQuantity, public ColorQuantity<SurfaceColorQuantity> {
```
Class SurfaceColorQuantity.

```cpp
SurfaceColorQuantity(std::string name, SurfaceMesh& mesh_, std::string definedOn, const std::vector<glm::vec3>& colorValues);
```
Surface color quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
const std::string definedOn;
```
Member defined on.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
virtual void createProgram() = 0;
```
Create program.

```cpp
class SurfaceVertexColorQuantity : public SurfaceColorQuantity {
```
======================================================== ========== Vertex Color ========== ========================================================

```cpp
SurfaceVertexColorQuantity(std::string name, SurfaceMesh& mesh_, std::vector<glm::vec3> values_);
```
Surface vertex color quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual void buildColorOptionsUI() override;
```
Build color options ui.

```cpp
void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
class SurfaceFaceColorQuantity : public SurfaceColorQuantity {
```
======================================================== ========== Face Color ========== ========================================================

```cpp
SurfaceFaceColorQuantity(std::string name, SurfaceMesh& mesh_, std::vector<glm::vec3> values_);
```
Surface face color quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual void buildColorOptionsUI() override;
```
Build color options ui.

```cpp
void buildFaceInfoGUI(size_t fInd) override;
```
Build face info gui.

```cpp
class SurfaceTextureColorQuantity : public SurfaceColorQuantity, class SurfaceTextureColorQuantity : public SurfaceColorQuantity, public TextureMapQuantity<SurfaceTextureColorQuantity> {
```
Class SurfaceTextureColorQuantity.

```cpp
SurfaceTextureColorQuantity(std::string name, SurfaceMesh& mesh_, SurfaceParameterizationQuantity& param_, size_t dimX, size_t dimY, std::vector<glm::vec3> values_, ImageOrigin origin_);
```
Surface texture color quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual void buildColorOptionsUI() override;
```
Build color options ui.

```cpp
SurfaceParameterizationQuantity& param;
```
Member param.

## surface_mesh.h

```cpp
class SurfaceVertexColorQuantity;
```
Forward declarations for quantities

```cpp
class SurfaceFaceColorQuantity;
```
Class SurfaceFaceColorQuantity.

```cpp
class SurfaceTextureColorQuantity;
```
Class SurfaceTextureColorQuantity.

```cpp
class SurfaceScalarQuantity;
```
Class SurfaceScalarQuantity.

```cpp
class SurfaceVertexScalarQuantity;
```
Class SurfaceVertexScalarQuantity.

```cpp
class SurfaceFaceScalarQuantity;
```
Class SurfaceFaceScalarQuantity.

```cpp
class SurfaceEdgeScalarQuantity;
```
Class SurfaceEdgeScalarQuantity.

```cpp
class SurfaceHalfedgeScalarQuantity;
```
Class SurfaceHalfedgeScalarQuantity.

```cpp
class SurfaceVertexScalarQuantity;
```
Class SurfaceVertexScalarQuantity.

```cpp
class SurfaceCornerScalarQuantity;
```
Class SurfaceCornerScalarQuantity.

```cpp
class SurfaceTextureScalarQuantity;
```
Class SurfaceTextureScalarQuantity.

```cpp
class SurfaceCornerParameterizationQuantity;
```
Class SurfaceCornerParameterizationQuantity.

```cpp
class SurfaceVertexParameterizationQuantity;
```
Class SurfaceVertexParameterizationQuantity.

```cpp
class SurfaceVertexVectorQuantity;
```
Class SurfaceVertexVectorQuantity.

```cpp
class SurfaceFaceVectorQuantity;
```
Class SurfaceFaceVectorQuantity.

```cpp
class SurfaceVertexTangentVectorQuantity;
```
Class SurfaceVertexTangentVectorQuantity.

```cpp
class SurfaceFaceTangentVectorQuantity;
```
Class SurfaceFaceTangentVectorQuantity.

```cpp
class SurfaceOneFormTangentVectorQuantity;
```
Class SurfaceOneFormTangentVectorQuantity.

```cpp
template <> // Specialize the quantity type struct QuantityTypeHelper<SurfaceMesh> {
```
Struct QuantityTypeHelper.

```cpp
struct SurfaceMeshPickResult {
```
Struct SurfaceMeshPickResult.

```cpp
MeshElement elementType;
```
Member element type.

```cpp
int64_t index;
```
Member index.

```cpp
class SurfaceMesh : public QuantityStructure<SurfaceMesh> {
```
=== The grand surface mesh class

```cpp
SurfaceMesh(std::string name);
```
Surface mesh.

```cpp
SurfaceMesh(std::string name, const std::vector<glm::vec3>& vertexPositions, const std::vector<uint32_t>& faceIndsEntries, const std::vector<uint32_t>& faceIndsStart);
```
Surface mesh.

```cpp
SurfaceMesh(std::string name, const std::vector<glm::vec3>& vertexPositions, const std::vector<std::vector<size_t>>& faceIndices);
```
Surface mesh.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void buildPickUI(const PickResult&) override;
```
Build pick ui.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
std::vector<uint32_t> faceIndsStart;
```
Member face inds start.

```cpp
std::vector<uint32_t> faceIndsEntries;
```
Member face inds entries.

```cpp
render::ManagedBuffer<glm::vec3> vertexPositions;
```
Member vertex positions.

```cpp
render::ManagedBuffer<uint32_t> triangleVertexInds;
```
Member triangle vertex inds.

```cpp
render::ManagedBuffer<uint32_t> triangleFaceInds;
```
Member triangle face inds.

```cpp
render::ManagedBuffer<uint32_t> triangleCornerInds;
```
Member triangle corner inds.

```cpp
render::ManagedBuffer<uint32_t> triangleAllVertexInds;
```
Member triangle all vertex inds.

```cpp
render::ManagedBuffer<uint32_t> triangleAllEdgeInds;
```
Member triangle all edge inds.

```cpp
render::ManagedBuffer<uint32_t> triangleAllHalfedgeInds;
```
Member triangle all halfedge inds.

```cpp
render::ManagedBuffer<uint32_t> triangleAllCornerInds;
```
Member triangle all corner inds.

```cpp
render::ManagedBuffer<glm::vec3> baryCoord;
```
Member bary coord.

```cpp
render::ManagedBuffer<glm::vec3> edgeIsReal;
```
Member edge is real.

```cpp
render::ManagedBuffer<glm::vec3> faceNormals;
```
Member face normals.

```cpp
render::ManagedBuffer<glm::vec3> faceCenters;
```
Member face centers.

```cpp
render::ManagedBuffer<float> faceAreas;
```
Member face areas.

```cpp
render::ManagedBuffer<glm::vec3> vertexNormals;
```
Member vertex normals.

```cpp
render::ManagedBuffer<float> vertexAreas;
```
Member vertex areas.

```cpp
render::ManagedBuffer<glm::vec3> defaultFaceTangentBasisX;
```
Member default face tangent basis x.

```cpp
render::ManagedBuffer<glm::vec3> defaultFaceTangentBasisY;
```
Member default face tangent basis y.

```cpp
template <class T> SurfaceVertexScalarQuantity* addVertexScalarQuantity(std::string name, const T& data, DataType type = DataType::STANDARD);
```
Add vertex scalar quantity.

```cpp
template <class T> SurfaceFaceScalarQuantity* addFaceScalarQuantity(std::string name, const T& data, DataType type = DataType::STANDARD);
```
Add face scalar quantity.

```cpp
template <class T> SurfaceEdgeScalarQuantity* addEdgeScalarQuantity(std::string name, const T& data, DataType type = DataType::STANDARD);
```
Add edge scalar quantity.

```cpp
template <class T> SurfaceHalfedgeScalarQuantity* addHalfedgeScalarQuantity(std::string name, const T& data, DataType type = DataType::STANDARD);
```
Add halfedge scalar quantity.

```cpp
template <class T> SurfaceCornerScalarQuantity* addCornerScalarQuantity(std::string name, const T& data, DataType type = DataType::STANDARD);
```
Add corner scalar quantity.

```cpp
template <class T> SurfaceTextureScalarQuantity* addTextureScalarQuantity(std::string name, SurfaceParameterizationQuantity& param, size_t dimX, size_t dimY, const T& data, ImageOrigin imageOrigin, DataType type = DataType::STANDARD);
```
Add texture scalar quantity.

```cpp
template <class T> SurfaceTextureScalarQuantity* addTextureScalarQuantity(std::string name, std::string paramName, size_t dimX, size_t dimY, const T& data, ImageOrigin imageOrigin, DataType type = DataType::STANDARD);
```
Add texture scalar quantity.

```cpp
template <class T> SurfaceVertexScalarQuantity* addVertexDistanceQuantity(std::string name, const T& data);
```
Add vertex distance quantity.

```cpp
template <class T> SurfaceVertexScalarQuantity* addVertexSignedDistanceQuantity(std::string name, const T& data);
```
Add vertex signed distance quantity.

```cpp
template <class T> SurfaceVertexColorQuantity* addVertexColorQuantity(std::string name, const T& data);
```
Add vertex color quantity.

```cpp
template <class T> SurfaceFaceColorQuantity* addFaceColorQuantity(std::string name, const T& data);
```
Add face color quantity.

```cpp
template <class T> SurfaceTextureColorQuantity* addTextureColorQuantity(std::string name, SurfaceParameterizationQuantity& param, size_t dimX, size_t dimY, const T& colors, ImageOrigin imageOrigin);
```
Add texture color quantity.

```cpp
template <class T> SurfaceTextureColorQuantity* addTextureColorQuantity(std::string name, std::string paramName, size_t dimX, size_t dimY, const T& colors, ImageOrigin imageOrigin);
```
Add texture color quantity.

```cpp
template <class T> SurfaceCornerParameterizationQuantity* addParameterizationQuantity(std::string name, const T& coords, ParamCoordsType type = ParamCoordsType::UNIT);
```
Add parameterization quantity.

```cpp
template <class T> SurfaceVertexParameterizationQuantity* addVertexParameterizationQuantity(std::string name, const T& coords, ParamCoordsType type = ParamCoordsType::UNIT);
```
Add vertex parameterization quantity.

```cpp
template <class T> SurfaceVertexParameterizationQuantity* addLocalParameterizationQuantity(std::string name, const T& coords, ParamCoordsType type = ParamCoordsType::WORLD);
```
Add local parameterization quantity.

```cpp
template <class T> SurfaceVertexVectorQuantity* addVertexVectorQuantity(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add vertex vector quantity.

```cpp
template <class T> SurfaceVertexVectorQuantity* addVertexVectorQuantity2D(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add vertex vector quantity 2 d.

```cpp
template <class T> SurfaceFaceVectorQuantity* addFaceVectorQuantity(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add face vector quantity.

```cpp
template <class T> SurfaceFaceVectorQuantity* addFaceVectorQuantity2D(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add face vector quantity 2 d.

```cpp
template <class T, class BX, class BY> SurfaceFaceTangentVectorQuantity* addFaceTangentVectorQuantity(std::string name, const T& vectors, const BX& basisX, const BY& basisY, int nSym = 1, VectorType vectorType = VectorType::STANDARD);
```
Add face tangent vector quantity.

```cpp
template <class T, class BX, class BY> SurfaceVertexTangentVectorQuantity* addVertexTangentVectorQuantity(std::string name, const T& vectors, const BX& basisX, const BY& basisY, int nSym = 1, VectorType vectorType = VectorType::STANDARD);
```
Add vertex tangent vector quantity.

```cpp
template <class T, class O> SurfaceOneFormTangentVectorQuantity* addOneFormTangentVectorQuantity(std::string name, const T& data, const O& orientations);
```
Add one form tangent vector quantity.

```cpp
SurfaceParameterizationQuantity* getParameterization(std::string name);
```
Get parameterization.

```cpp
SurfaceMeshPickResult interpretPickResult(const PickResult& result);
```
Interpret pick result.

```cpp
long long int selectVertex();
```
Select vertex.

```cpp
template <class V> void updateVertexPositions(const V& newPositions);
```
Update vertex positions.

```cpp
template <class V> void updateVertexPositions2D(const V& newPositions2D);
```
Update vertex positions 2 d.

```cpp
void setTransparencyQuantity(SurfaceScalarQuantity* quantity);
```
Set transparency quantity.

```cpp
void setTransparencyQuantity(std::string name);
```
Set transparency quantity.

```cpp
void clearTransparencyQuantity();
```
Clear transparency quantity.

```cpp
std::vector<size_t> edgePerm;
```
Member edge perm.

```cpp
std::vector<size_t> halfedgePerm;
```
Member halfedge perm.

```cpp
std::vector<size_t> cornerPerm;
```
Member corner perm.

```cpp
template <class T> void setEdgePermutation(const T& perm, size_t expectedSize = 0);
```
Set edge permutation.

```cpp
template <class T> void setHalfedgePermutation(const T& perm, size_t expectedSize = 0);
```
Set halfedge permutation.

```cpp
template <class T> void setCornerPermutation(const T& perm, size_t expectedSize = 0);
```
Set corner permutation.

```cpp
template <class T> void setAllPermutations(const std::array<std::pair<T, size_t>, 3>& perms);
```
Set all permutations.

```cpp
template <class T> void setAllPermutations(const std::array<std::pair<T, size_t>, 5>& perms);
```
Set all permutations.

```cpp
size_t vertexDataSize = INVALID_IND;
```
Member invalid ind.

```cpp
size_t faceDataSize = INVALID_IND;
```
Member invalid ind.

```cpp
size_t edgeDataSize = INVALID_IND;
```
Member invalid ind.

```cpp
size_t halfedgeDataSize = INVALID_IND;
```
Member invalid ind.

```cpp
size_t cornerDataSize = INVALID_IND;
```
Member invalid ind.

```cpp
size_t nVertices();
```
N vertices.

```cpp
size_t nFaces() const { return faceIndsStart.size() - 1; }
```
N faces.

```cpp
size_t nFacesTriangulation() const { return nFacesTriangulationCount; }
```
N faces triangulation.

```cpp
size_t nEdgesCount = INVALID_IND;
```
Member invalid ind.

```cpp
size_t nEdges();
```
N edges.

```cpp
size_t nCorners() const { return nCornersCount; }
```
N corners.

```cpp
size_t nHalfedges() const { return nCornersCount; }
```
N halfedges.

```cpp
void nestedFacesToFlat(const std::vector<std::vector<size_t>>& nestedInds);
```
Nested faces to flat.

```cpp
void computeConnectivityData();
```
Compute connectivity data.

```cpp
void checkTriangular();
```
Check triangular.

```cpp
void markEdgesAsUsed();
```
Mark edges as used.

```cpp
void markHalfedgesAsUsed();
```
Mark halfedges as used.

```cpp
void markCornersAsUsed();
```
Mark corners as used.

```cpp
void ensureHaveManifoldConnectivity();
```
Ensure have manifold connectivity.

```cpp
std::vector<size_t> twinHalfedge;
```
Member twin halfedge.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
SurfaceMesh* setSurfaceColor(glm::vec3 val);
```
Set surface color.

```cpp
glm::vec3 getSurfaceColor();
```
Get surface color.

```cpp
SurfaceMesh* setEdgeColor(glm::vec3 val);
```
Set edge color.

```cpp
glm::vec3 getEdgeColor();
```
Get edge color.

```cpp
SurfaceMesh* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
SurfaceMesh* setBackFaceColor(glm::vec3 val);
```
Set back face color.

```cpp
glm::vec3 getBackFaceColor();
```
Get back face color.

```cpp
SurfaceMesh* setEdgeWidth(double newVal);
```
Set edge width.

```cpp
double getEdgeWidth();
```
Get edge width.

```cpp
SurfaceMesh* setBackFacePolicy(BackFacePolicy newPolicy);
```
Set back face policy.

```cpp
BackFacePolicy getBackFacePolicy();
```
Get back face policy.

```cpp
SurfaceMesh* setShadeStyle(MeshShadeStyle newStyle);
```
Set shade style.

```cpp
MeshShadeStyle getShadeStyle();
```
Get shade style.

```cpp
SurfaceMesh* setSelectionMode(MeshSelectionMode newMode);
```
Set selection mode.

```cpp
MeshSelectionMode getSelectionMode();
```
Get selection mode.

```cpp
std::vector<std::string> addSurfaceMeshRules(std::vector<std::string> initRules, bool withMesh = true, bool withSurfaceShade = true);
```
Add surface mesh rules.

```cpp
void setMeshGeometryAttributes(render::ShaderProgram& p);
```
Set mesh geometry attributes.

```cpp
void setMeshPickAttributes(render::ShaderProgram& p);
```
Set mesh pick attributes.

```cpp
void setSurfaceMeshUniforms(render::ShaderProgram& p);
```
Set surface mesh uniforms.

```cpp
SurfaceMesh* setSmoothShade(bool isSmooth);
```
Set smooth shade.

```cpp
bool isSmoothShade();
```
Return whether smooth shade.

```cpp
std::vector<glm::vec3> vertexPositionsData;
```
Member vertex positions data.

```cpp
std::vector<uint32_t> triangleVertexIndsData;
```
Member triangle vertex inds data.

```cpp
std::vector<uint32_t> triangleFaceIndsData;
```
Member triangle face inds data.

```cpp
std::vector<uint32_t> triangleCornerIndsData;
```
Member triangle corner inds data.

```cpp
std::vector<uint32_t> triangleAllVertexIndsData;
```
Member triangle all vertex inds data.

```cpp
std::vector<uint32_t> triangleAllEdgeIndsData;
```
Member triangle all edge inds data.

```cpp
std::vector<uint32_t> triangleAllHalfedgeIndsData;
```
Member triangle all halfedge inds data.

```cpp
std::vector<uint32_t> triangleAllCornerIndsData;
```
Member triangle all corner inds data.

```cpp
std::vector<glm::vec3> baryCoordData;
```
Member bary coord data.

```cpp
std::vector<glm::vec3> edgeIsRealData;
```
Member edge is real data.

```cpp
std::vector<glm::vec3> faceNormalsData;
```
Member face normals data.

```cpp
std::vector<glm::vec3> faceCentersData;
```
Member face centers data.

```cpp
std::vector<float> faceAreasData;
```
Member face areas data.

```cpp
std::vector<glm::vec3> vertexNormalsData;
```
Member vertex normals data.

```cpp
std::vector<float> vertexAreasData;
```
Member vertex areas data.

```cpp
std::vector<glm::vec3> defaultFaceTangentBasisXData;
```
Member default face tangent basis x data.

```cpp
std::vector<glm::vec3> defaultFaceTangentBasisYData;
```
Member default face tangent basis y data.

```cpp
bool halfedgesHaveBeenUsed = false;
```
Member false.

```cpp
bool cornersHaveBeenUsed = false;
```
Member false.

```cpp
bool edgesHaveBeenUsed = false;
```
Member false.

```cpp
std::vector<uint32_t> halfedgeEdgeCorrespondence;
```
Member halfedge edge correspondence.

```cpp
PersistentValue<glm::vec3> surfaceColor;
```
Member surface color.

```cpp
PersistentValue<glm::vec3> edgeColor;
```
Member edge color.

```cpp
PersistentValue<std::string> material;
```
Member material.

```cpp
PersistentValue<float> edgeWidth;
```
Member edge width.

```cpp
PersistentValue<BackFacePolicy> backFacePolicy;
```
Member back face policy.

```cpp
PersistentValue<glm::vec3> backFaceColor;
```
Member back face color.

```cpp
PersistentValue<MeshShadeStyle> shadeStyle;
```
Member shade style.

```cpp
PersistentValue<MeshSelectionMode> selectionMode;
```
Member selection mode.

```cpp
void prepare();
```
Prepare.

```cpp
void preparePick();
```
Prepare pick.

```cpp
void computeTriangleCornerInds();
```
Compute triangle corner inds.

```cpp
void computeTriangleAllVertexInds();
```
Compute triangle all vertex inds.

```cpp
void computeTriangleAllEdgeInds();
```
Compute triangle all edge inds.

```cpp
void computeTriangleAllHalfedgeInds();
```
Compute triangle all halfedge inds.

```cpp
void computeTriangleAllCornerInds();
```
Compute triangle all corner inds.

```cpp
void computeFaceNormals();
```
Compute face normals.

```cpp
void computeFaceCenters();
```
Compute face centers.

```cpp
void computeFaceAreas();
```
Compute face areas.

```cpp
void computeVertexNormals();
```
Compute vertex normals.

```cpp
void computeVertexAreas();
```
Compute vertex areas.

```cpp
void computeEdgeLengths();
```
Compute edge lengths.

```cpp
void computeDefaultFaceTangentBasisX();
```
Compute default face tangent basis x.

```cpp
void computeDefaultFaceTangentBasisY();
```
Compute default face tangent basis y.

```cpp
void countEdges();
```
Count edges.

```cpp
size_t facePickIndStart, edgePickIndStart, halfedgePickIndStart, cornerPickIndStart;
```
Member face pick ind start.

```cpp
size_t facePickIndStart, edgePickIndStart, halfedgePickIndStart, cornerPickIndStart;
```
Member edge pick ind start.

```cpp
size_t facePickIndStart, edgePickIndStart, halfedgePickIndStart, cornerPickIndStart;
```
Member halfedge pick ind start.

```cpp
size_t facePickIndStart, edgePickIndStart, halfedgePickIndStart, cornerPickIndStart;
```
Member corner pick ind start.

```cpp
void buildVertexInfoGui(const SurfaceMeshPickResult& result);
```
Build vertex info gui.

```cpp
void buildFaceInfoGui(const SurfaceMeshPickResult& result);
```
Build face info gui.

```cpp
void buildEdgeInfoGui(const SurfaceMeshPickResult& result);
```
Build edge info gui.

```cpp
void buildHalfedgeInfoGui(const SurfaceMeshPickResult& result);
```
Build halfedge info gui.

```cpp
void buildCornerInfoGui(const SurfaceMeshPickResult& result);
```
Build corner info gui.

```cpp
SurfaceScalarQuantity& resolveTransparencyQuantity();
```
Resolve transparency quantity.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
std::shared_ptr<render::ShaderProgram> pickProgram;
```
Member pick program.

```cpp
bool usingSimplePick = false;
```
Member false.

```cpp
void initializeMeshTriangulation();
```
Initialize mesh triangulation.

```cpp
void recomputeGeometryIfPopulated();
```
Recompute geometry if populated.

```cpp
glm::vec2 projectToScreenSpace(glm::vec3 coord);
```
Project to screen space.

```cpp
SurfaceVertexColorQuantity* addVertexColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
```
Add vertex color quantity impl.

```cpp
SurfaceFaceColorQuantity* addFaceColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
```
Add face color quantity impl.

```cpp
SurfaceTextureColorQuantity* addTextureColorQuantityImpl(std::string name, SurfaceParameterizationQuantity& param, size_t dimX, size_t dimY, const std::vector<glm::vec3>& colors, ImageOrigin imageOrigin);
```
Add texture color quantity impl.

```cpp
SurfaceVertexScalarQuantity* addVertexScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add vertex scalar quantity impl.

```cpp
SurfaceFaceScalarQuantity* addFaceScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add face scalar quantity impl.

```cpp
SurfaceEdgeScalarQuantity* addEdgeScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add edge scalar quantity impl.

```cpp
SurfaceHalfedgeScalarQuantity* addHalfedgeScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add halfedge scalar quantity impl.

```cpp
SurfaceCornerScalarQuantity* addCornerScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add corner scalar quantity impl.

```cpp
SurfaceTextureScalarQuantity* addTextureScalarQuantityImpl(std::string name, SurfaceParameterizationQuantity& param, size_t dimX, size_t dimY, const std::vector<float>& data, ImageOrigin imageOrigin, DataType type);
```
Add texture scalar quantity impl.

```cpp
SurfaceVertexScalarQuantity* addVertexDistanceQuantityImpl(std::string name, const std::vector<float>& data);
```
Add vertex distance quantity impl.

```cpp
SurfaceVertexScalarQuantity* addVertexSignedDistanceQuantityImpl(std::string name, const std::vector<float>& data);
```
Add vertex signed distance quantity impl.

```cpp
SurfaceCornerParameterizationQuantity* addParameterizationQuantityImpl(std::string name, const std::vector<glm::vec2>& coords, ParamCoordsType type);
```
Add parameterization quantity impl.

```cpp
SurfaceVertexParameterizationQuantity* addVertexParameterizationQuantityImpl(std::string name, const std::vector<glm::vec2>& coords, ParamCoordsType type);
```
Add vertex parameterization quantity impl.

```cpp
SurfaceVertexParameterizationQuantity* addLocalParameterizationQuantityImpl(std::string name, const std::vector<glm::vec2>& coords, ParamCoordsType type);
```
Add local parameterization quantity impl.

```cpp
SurfaceVertexVectorQuantity* addVertexVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors, VectorType vectorType);
```
Add vertex vector quantity impl.

```cpp
SurfaceFaceVectorQuantity* addFaceVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors, VectorType vectorType);
```
Add face vector quantity impl.

```cpp
SurfaceFaceTangentVectorQuantity* addFaceTangentVectorQuantityImpl(std::string name, const std::vector<glm::vec2>& vectors, const std::vector<glm::vec3>& basisX, const std::vector<glm::vec3>& basisY, int nSym, VectorType vectorType);
```
Add face tangent vector quantity impl.

```cpp
SurfaceVertexTangentVectorQuantity* addVertexTangentVectorQuantityImpl(std::string name, const std::vector<glm::vec2>& vectors, const std::vector<glm::vec3>& basisX, const std::vector<glm::vec3>& basisY, int nSym, VectorType vectorType);
```
Add vertex tangent vector quantity impl.

```cpp
SurfaceOneFormTangentVectorQuantity* addOneFormTangentVectorQuantityImpl(std::string name, const std::vector<float>& data, const std::vector<char>& orientations);
```
Add one form tangent vector quantity impl.

```cpp
template <class V, class F> SurfaceMesh* registerSurfaceMesh(std::string name, const V& vertexPositions, const F& faceIndices);
```
Register surface mesh.

```cpp
template <class V, class F> template <class V, class F> SurfaceMesh* registerSurfaceMesh2D(std::string name, const V& vertexPositions, const F& faceIndices);
```
Register surface mesh 2 d.

```cpp
template <class V, class F> template <class V, class F, class P> SurfaceMesh* registerSurfaceMesh(std::string name, const V& vertexPositions, const F& faceIndices, const std::array<std::pair<P, size_t>, 3>& perms);
```
Register surface mesh.

```cpp
template <class V, class F, class P> SurfaceMesh* registerSurfaceMesh(std::string name, const V& vertexPositions, const F& faceIndices, const std::array<std::pair<P, size_t>, 5>& perms);
```
Register surface mesh.

```cpp
inline SurfaceMesh* getSurfaceMesh(std::string name = "");
```
Get surface mesh.

```cpp
inline bool hasSurfaceMesh(std::string name = "");
```
Return whether it has surface mesh.

```cpp
inline void removeSurfaceMesh(std::string name = "", bool errorIfAbsent = false);
```
Remove surface mesh.

## surface_mesh_quantity.h

```cpp
class SurfaceMesh;
```
Forward delcare surface mesh

```cpp
class SurfaceMeshQuantity : public QuantityS<SurfaceMesh> {
```
Extend Quantity<SurfaceMesh> to add a few extra functions

```cpp
SurfaceMeshQuantity(std::string name, SurfaceMesh& parentStructure, bool dominates = false);
```
Surface mesh quantity.

```cpp
~SurfaceMeshQuantity() {};
```
Surface mesh quantity.

```cpp
virtual void buildVertexInfoGUI(size_t vInd);
```
Build vertex info gui.

```cpp
virtual void buildFaceInfoGUI(size_t fInd);
```
Build face info gui.

```cpp
virtual void buildEdgeInfoGUI(size_t eInd);
```
Build edge info gui.

```cpp
virtual void buildHalfedgeInfoGUI(size_t heInd);
```
Build halfedge info gui.

```cpp
virtual void buildCornerInfoGUI(size_t heInd);
```
Build corner info gui.

## surface_parameterization_quantity.h

```cpp
class CurveNetwork;
```
forward declarations

```cpp
class SurfaceParameterizationQuantity : public SurfaceMeshQuantity, class SurfaceParameterizationQuantity : public SurfaceMeshQuantity, public ParameterizationQuantity<SurfaceParameterizationQuantity> {
```
Class SurfaceParameterizationQuantity.

```cpp
SurfaceParameterizationQuantity(std::string name, SurfaceMesh& mesh_, const std::vector<glm::vec2>& coords_, MeshElement definedOn, ParamCoordsType type_, ParamVizStyle style_);
```
Surface parameterization quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
const MeshElement definedOn;
```
Member defined on.

```cpp
template <typename V> void setIslandLabels(const V& newIslandLabels);
```
Set island labels.

```cpp
CurveNetwork* createCurveNetworkFromSeams(std::string structureName = "");
```
Create curve network from seams.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
void createProgram();
```
Create program.

```cpp
size_t nFaces();
```
N faces.

```cpp
virtual void fillCoordBuffers(render::ShaderProgram& p) = 0;
```
Fill coord buffers.

```cpp
class SurfaceCornerParameterizationQuantity : public SurfaceParameterizationQuantity {
```
============================================================== =============== Corner Parameterization ==================== ==============================================================

```cpp
SurfaceCornerParameterizationQuantity(std::string name, SurfaceMesh& mesh_, const std::vector<glm::vec2>& coords_, ParamCoordsType type_, ParamVizStyle style);
```
Surface corner parameterization quantity.

```cpp
virtual void buildCornerInfoGUI(size_t cInd) override;
```
Build corner info gui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void fillCoordBuffers(render::ShaderProgram& p) override;
```
Fill coord buffers.

```cpp
class SurfaceVertexParameterizationQuantity : public SurfaceParameterizationQuantity {
```
============================================================== =============== Vertex Parameterization ==================== ==============================================================

```cpp
SurfaceVertexParameterizationQuantity(std::string name, SurfaceMesh& mesh_, const std::vector<glm::vec2>& coords_, ParamCoordsType type_, ParamVizStyle style);
```
Surface vertex parameterization quantity.

```cpp
virtual void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void fillCoordBuffers(render::ShaderProgram& p) override;
```
Fill coord buffers.

```cpp
template <typename V> void SurfaceParameterizationQuantity::setIslandLabels(const V& newIslandLabels) { validateSize(newIslandLabels, this->nFaces(), "scalar quantity " + quantity.name);
```
Set island labels.

```cpp
islandLabels.markHostBufferUpdated();
```
Mark host buffer updated.

## surface_scalar_quantity.h

```cpp
class SurfaceMeshQuantity;
```
forward declarations

```cpp
class SurfaceMesh;
```
Class SurfaceMesh.

```cpp
class SurfaceParameterizationQuantity;
```
Class SurfaceParameterizationQuantity.

```cpp
class SurfaceScalarQuantity : public SurfaceMeshQuantity, public ScalarQuantity<SurfaceScalarQuantity> {
```
Class SurfaceScalarQuantity.

```cpp
SurfaceScalarQuantity(std::string name, SurfaceMesh& mesh_, std::string definedOn, const std::vector<float>& values_, DataType dataType);
```
Surface scalar quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildSurfaceScalarOptionsUI() {};
```
Build surface scalar options ui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::shared_ptr<render::AttributeBuffer> getAttributeBuffer() = 0;
```
Get attribute buffer.

```cpp
const std::string definedOn;
```
Member defined on.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
virtual void createProgram() = 0;
```
Create program.

```cpp
class SurfaceVertexScalarQuantity : public SurfaceScalarQuantity {
```
======================================================== ========== Vertex Scalar ========== ========================================================

```cpp
SurfaceVertexScalarQuantity(std::string name, const std::vector<float>& values_, SurfaceMesh& mesh_, DataType dataType_ = DataType::STANDARD);
```
Surface vertex scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual std::shared_ptr<render::AttributeBuffer> getAttributeBuffer() override;
```
Get attribute buffer.

```cpp
void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
class SurfaceFaceScalarQuantity : public SurfaceScalarQuantity {
```
======================================================== ========== Face Scalar ========== ========================================================

```cpp
SurfaceFaceScalarQuantity(std::string name, const std::vector<float>& values_, SurfaceMesh& mesh_, DataType dataType_ = DataType::STANDARD);
```
Surface face scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual std::shared_ptr<render::AttributeBuffer> getAttributeBuffer() override;
```
Get attribute buffer.

```cpp
void buildFaceInfoGUI(size_t fInd) override;
```
Build face info gui.

```cpp
class SurfaceEdgeScalarQuantity : public SurfaceScalarQuantity {
```
======================================================== ========== Edge Scalar ========== ========================================================

```cpp
SurfaceEdgeScalarQuantity(std::string name, const std::vector<float>& values_, SurfaceMesh& mesh_, DataType dataType_ = DataType::STANDARD);
```
Surface edge scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual std::shared_ptr<render::AttributeBuffer> getAttributeBuffer() override;
```
Get attribute buffer.

```cpp
void buildEdgeInfoGUI(size_t edgeInd) override;
```
Build edge info gui.

```cpp
class SurfaceHalfedgeScalarQuantity : public SurfaceScalarQuantity {
```
======================================================== ========== Halfedge Scalar ========== ========================================================

```cpp
SurfaceHalfedgeScalarQuantity(std::string name, const std::vector<float>& values_, SurfaceMesh& mesh_, DataType dataType_ = DataType::STANDARD);
```
Surface halfedge scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual std::shared_ptr<render::AttributeBuffer> getAttributeBuffer() override;
```
Get attribute buffer.

```cpp
void buildHalfedgeInfoGUI(size_t heInd) override;
```
Build halfedge info gui.

```cpp
class SurfaceCornerScalarQuantity : public SurfaceScalarQuantity {
```
======================================================== ========== Corner Scalar ========== ========================================================

```cpp
SurfaceCornerScalarQuantity(std::string name, const std::vector<float>& values_, SurfaceMesh& mesh_, DataType dataType_ = DataType::STANDARD);
```
Surface corner scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual std::shared_ptr<render::AttributeBuffer> getAttributeBuffer() override;
```
Get attribute buffer.

```cpp
void buildCornerInfoGUI(size_t heInd) override;
```
Build corner info gui.

```cpp
class SurfaceTextureScalarQuantity : public SurfaceScalarQuantity, class SurfaceTextureScalarQuantity : public SurfaceScalarQuantity, public TextureMapQuantity<SurfaceTextureScalarQuantity> {
```
Class SurfaceTextureScalarQuantity.

```cpp
SurfaceTextureScalarQuantity(std::string name, SurfaceMesh& mesh_, SurfaceParameterizationQuantity& param_, size_t dimX, size_t dimY, const std::vector<float>& values_, ImageOrigin origin_, DataType dataType_ = DataType::STANDARD);
```
Surface texture scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual void buildSurfaceScalarOptionsUI() override;
```
Build surface scalar options ui.

```cpp
virtual std::shared_ptr<render::AttributeBuffer> getAttributeBuffer() override;
```
Get attribute buffer.

```cpp
SurfaceParameterizationQuantity& param;
```
Member param.

## surface_vector_quantity.h

```cpp
class SurfaceVectorQuantity : public SurfaceMeshQuantity {
```
==== Common base class Represents a general vector field associated with a surface mesh, including R3 fields in the ambient space and R2 fields embedded in the surface NOTE: This intermediate class is not really necessary anymore; it is subsumed by the VectorQuantity<> classes which serve as common bases for ALL vector types.

```cpp
SurfaceVectorQuantity(std::string name, SurfaceMesh& mesh_, MeshElement definedOn_);
```
Surface vector quantity.

```cpp
MeshElement definedOn;
```
Member defined on.

```cpp
class SurfaceVertexVectorQuantity : public SurfaceVectorQuantity, public VectorQuantity<SurfaceVertexVectorQuantity> {
```
==== R3 vectors at vertices

```cpp
SurfaceVertexVectorQuantity(std::string name, std::vector<glm::vec3> vectors_, SurfaceMesh& mesh_, VectorType vectorType_ = VectorType::STANDARD);
```
Surface vertex vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
class SurfaceFaceVectorQuantity : public SurfaceVectorQuantity, public VectorQuantity<SurfaceFaceVectorQuantity> {
```
==== R3 vectors at faces

```cpp
SurfaceFaceVectorQuantity(std::string name, std::vector<glm::vec3> vectors_, SurfaceMesh& mesh_, VectorType vectorType_ = VectorType::STANDARD);
```
Surface face vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void buildFaceInfoGUI(size_t fInd) override;
```
Build face info gui.

```cpp
class SurfaceFaceTangentVectorQuantity : public SurfaceVectorQuantity, class SurfaceFaceTangentVectorQuantity : public SurfaceVectorQuantity, public TangentVectorQuantity<SurfaceFaceTangentVectorQuantity> {
```
Class SurfaceFaceTangentVectorQuantity.

```cpp
SurfaceFaceTangentVectorQuantity(std::string name, std::vector<glm::vec2> vectors_, std::vector<glm::vec3> basisX_, std::vector<glm::vec3> basisY_, SurfaceMesh& mesh_, int nSym = 1, VectorType vectorType_ = VectorType::STANDARD);
```
Surface face tangent vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
void buildFaceInfoGUI(size_t fInd) override;
```
Build face info gui.

```cpp
class SurfaceVertexTangentVectorQuantity : public SurfaceVectorQuantity, class SurfaceVertexTangentVectorQuantity : public SurfaceVectorQuantity, public TangentVectorQuantity<SurfaceVertexTangentVectorQuantity> {
```
Class SurfaceVertexTangentVectorQuantity.

```cpp
SurfaceVertexTangentVectorQuantity(std::string name, std::vector<glm::vec2> vectors_, std::vector<glm::vec3> basisX_, std::vector<glm::vec3> basisY_, SurfaceMesh& mesh_, int nSym = 1, VectorType vectorType_ = VectorType::STANDARD);
```
Surface vertex tangent vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
class SurfaceOneFormTangentVectorQuantity : public SurfaceVectorQuantity, class SurfaceOneFormTangentVectorQuantity : public SurfaceVectorQuantity, public TangentVectorQuantity<SurfaceOneFormTangentVectorQuantity> {
```
Class SurfaceOneFormTangentVectorQuantity.

```cpp
SurfaceOneFormTangentVectorQuantity(std::string name, std::vector<float> oneForm_, std::vector<char> orientations_, SurfaceMesh& mesh_);
```
Surface one form tangent vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
std::vector<float> oneForm;
```
Member one form.

```cpp
std::vector<char> canonicalOrientation;
```
Member canonical orientation.

```cpp
void buildEdgeInfoGUI(size_t eInd) override;
```
Build edge info gui.

## texture_map_quantity.h

```cpp
template <typename QuantityT> class TextureMapQuantity {
```
Class TextureMapQuantity.

```cpp
TextureMapQuantity(QuantityT& parent, size_t dimX, size_t dimY, ImageOrigin origin_);
```
Texture map quantity.

```cpp
virtual void buildTextureMapOptionsUI();
```
Build texture map options ui.

```cpp
QuantityT& quantity;
```
Member quantity.

```cpp
QuantityT* setFilterMode(FilterMode newFilterMode);
```
Set filter mode.

```cpp
FilterMode getFilterMode();
```
Get filter mode.

```cpp
size_t dimX, dimY;
```
Member dim x.

```cpp
size_t dimX, dimY;
```
Member dim y.

```cpp
ImageOrigin imageOrigin;
```
Member image origin.

```cpp
PersistentValue<FilterMode> filterMode;
```
Member filter mode.

## transformation_gizmo.h

```cpp
class TransformationGizmo : public Widget {
```
A visual widget with handles for translations/rotations

```cpp
TransformationGizmo(std::string name, glm::mat4& T, PersistentValue<glm::mat4>* Tpers = nullptr);
```
Transformation gizmo.

```cpp
const std::string name;
```
Member name.

```cpp
PersistentValue<bool> enabled;
```
Member enabled.

```cpp
glm::mat4& T;
```
Member t.

```cpp
PersistentValue<glm::mat4>* Tpers;
```
Member tpers.

```cpp
void prepare();
```
Prepare.

```cpp
void draw() override;
```
Draw.

```cpp
bool interact() override;
```
Interact.

```cpp
enum class TransformHandle { None, Rotation, Translation, Scale };
```
Member none.

```cpp
enum class TransformHandle { None, Rotation, Translation, Scale };
```
Member rotation.

```cpp
enum class TransformHandle { None, Rotation, Translation, Scale };
```
Member translation.

```cpp
TransformHandle selectedType = TransformHandle::None;
```
Member none.

```cpp
bool currentlyDragging = false;
```
Member false.

```cpp
std::array<glm::vec3, 3> niceRGB = {{glm::vec3{211 / 255., 45 / 255., 62 / 255.}, glm::vec3{65 / 255., 121 / 255., 225 / 255.}, glm::vec3{95 / 255., 175 / 255., 35 / 255.}}};
```
Member vec 3.

```cpp
void markUpdated();
```
Mark updated.

```cpp
std::shared_ptr<render::ShaderProgram> ringProgram;
```
Member ring program.

```cpp
std::shared_ptr<render::ShaderProgram> arrowProgram;
```
Member arrow program.

```cpp
std::shared_ptr<render::ShaderProgram> sphereProgram;
```
Member sphere program.

```cpp
std::tuple<float, float, glm::vec3> circleTest(glm::vec3 raySource, glm::vec3 rayDir, glm::vec3 center, glm::vec3 normal, float radius);
```
Circle test.

```cpp
std::tuple<float, float, glm::vec3> lineTest(glm::vec3 raySource, glm::vec3 rayDir, glm::vec3 center, glm::vec3 tangent, float length);
```
Line test.

```cpp
std::tuple<float, float, glm::vec3> sphereTest(glm::vec3 raySource, glm::vec3 rayDir, glm::vec3 center, float radius, bool allowHitSurface = true);
```
Sphere test.

```cpp
std::tuple<std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec2>, std::vector<glm::vec3>> triplePlaneCoords();
```
Triple plane coords.

```cpp
std::tuple<std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec3>> tripleArrowCoords();
```
Triple arrow coords.

## types.h

```cpp
enum class NavigateStyle { Turntable = 0, Free, Planar, Arcball, None, FirstPerson };
```
Enum NavigateStyle.

```cpp
enum class UpDir { XUp = 0, YUp, ZUp, NegXUp, NegYUp, NegZUp };
```
Enum UpDir.

```cpp
enum class FrontDir { XFront = 0, YFront, ZFront, NegXFront, NegYFront, NegZFront };
```
Enum FrontDir.

```cpp
enum class BackgroundView { None = 0 };
```
Enum BackgroundView.

```cpp
enum class ProjectionMode { Perspective = 0, Orthographic };
```
Enum ProjectionMode.

```cpp
enum class TransparencyMode { None = 0, Simple, Pretty };
```
Enum TransparencyMode.

```cpp
enum class GroundPlaneMode { None, Tile, TileReflection, ShadowOnly };
```
Enum GroundPlaneMode.

```cpp
enum class GroundPlaneHeightMode { Automatic = 0, Manual };
```
Enum GroundPlaneHeightMode.

```cpp
enum class BackFacePolicy { Identical, Different, Custom, Cull };
```
Enum BackFacePolicy.

```cpp
enum class PointRenderMode { Sphere = 0, Quad };
```
Enum PointRenderMode.

```cpp
enum class MeshElement { VERTEX = 0, FACE, EDGE, HALFEDGE, CORNER };
```
Enum MeshElement.

```cpp
enum class MeshShadeStyle { Smooth = 0, Flat, TriFlat };
```
Enum MeshShadeStyle.

```cpp
enum class MeshSelectionMode { Auto = 0, VerticesOnly, FacesOnly };
```
Enum MeshSelectionMode.

```cpp
enum class CurveNetworkElement { NODE = 0, EDGE };
```
Enum CurveNetworkElement.

```cpp
enum class VolumeMeshElement { VERTEX = 0, EDGE, FACE, CELL };
```
Enum VolumeMeshElement.

```cpp
enum class VolumeCellType { TET = 0, HEX };
```
Enum VolumeCellType.

```cpp
enum class VolumeGridElement { NODE = 0, CELL };
```
Enum VolumeGridElement.

```cpp
enum class IsolineStyle { Stripe = 0, Contour };
```
Enum IsolineStyle.

```cpp
enum class ImplicitRenderMode { SphereMarch, FixedStep };
```
Enum ImplicitRenderMode.

```cpp
enum class ImageOrigin { LowerLeft, UpperLeft };
```
Enum ImageOrigin.

```cpp
enum class FilterMode { Nearest = 0, Linear };
```
Enum FilterMode.

```cpp
enum class ParamCoordsType { UNIT = 0, WORLD }; // UNIT -> [0,1], WORLD -> length-valued enum class ParamVizStyle {
```
Enum ParamCoordsType.

```cpp
enum class ManagedBufferType {
```
Enum ManagedBufferType.

```cpp
enum class DataType { STANDARD = 0, SYMMETRIC, MAGNITUDE, CATEGORICAL };
```
What is the meaningful range of these values? Used to set meaningful colormaps STANDARD: [-inf, inf], zero does not mean anything special (ie, position) SYMMETRIC: [-inf, inf], zero is special (ie, net profit/loss) MAGNITUDE: [0, inf], zero is special (ie, length of a vector) CATEGORICAL: data is integers corresponding to labels, etc

## utilities.h

```cpp
template <typename T> void safeDelete(T*& x) { if (x != nullptr) { delete x;
```
Safe delete.

```cpp
template <typename T> void safeDeleteArray(T*& x) { if (x != nullptr) { delete[] x;
```
Safe delete array.

```cpp
std::string guessNiceNameFromPath(std::string fullname);
```
Guess nice name from path.

```cpp
void validateName(const std::string& name);
```
Validate name.

```cpp
std::string prettyPrintCount(size_t count);
```
Pretty print count.

```cpp
template <typename... Args> std::string str_printf(const std::string& format, Args... args) { size_t size = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
```
Str printf.

```cpp
std::unique_ptr<char[]> buf(new char[size]);
```
Buf.

```cpp
std::snprintf(buf.get(), size, format.c_str(), args...);
```
Snprintf.

```cpp
return std::string(buf.get(), buf.get() + size - 1);
```
String.

```cpp
std::tuple<std::string, std::string> splitExt(std::string f);
```
Split ext.

```cpp
inline glm::vec3 componentwiseMin(const glm::vec3& vA, const glm::vec3& vB) { return glm::vec3{std::min(vA.x, vB.x), std::min(vA.y, vB.y), std::min(vA.z, vB.z)};
```
Componentwise min.

```cpp
inline glm::vec3 componentwiseMax(const glm::vec3& vA, const glm::vec3& vB) { return glm::vec3{std::max(vA.x, vB.x), std::max(vA.y, vB.y), std::max(vA.z, vB.z)};
```
Componentwise max.

```cpp
inline glm::vec3 circularPermuteEntries(const glm::vec3& v) { // (could be prettier with swizzel) return glm::vec3{v.z, v.x, v.y};
```
Circular permute entries.

```cpp
void splitTransform(const glm::mat4& trans, glm::mat3x4& R, glm::vec3& T);
```
Split transform.

```cpp
glm::mat4 buildTransform(const glm::mat3x4& R, const glm::vec3& T);
```
Build transform.

```cpp
inline bool isFinite(glm::vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); } inline std::ostream& operator<<(std::ostream& output, const glm::vec2& v) { output << std::setprecision(std::numeric_limits<float>::max_digits10);
```
Return whether finite.

```cpp
inline std::ostream& operator<<(std::ostream& output, const glm::vec3& v) { output << std::setprecision(std::numeric_limits<float>::max_digits10);
```
Set precision.

```cpp
inline std::ostream& operator<<(std::ostream& output, const glm::vec4& v) { output << std::setprecision(std::numeric_limits<float>::max_digits10);
```
Set precision.

```cpp
inline std::string to_string(const glm::vec3& v) { std::stringstream buffer;
```
To string.

```cpp
return buffer.str();
```
Str.

```cpp
inline std::string to_string_short(const glm::vec3& v) { return str_printf("<%1.3f, %1.3f, %1.3f>", v[0], v[1], v[2]); } // === Index management const size_t INVALID_IND = std::numeric_limits<size_t>::max();
```
To string short.

```cpp
const uint32_t INVALID_IND_32 = std::numeric_limits<uint32_t>::max();
```
Max.

```cpp
const uint64_t INVALID_IND_64 = std::numeric_limits<uint64_t>::max();
```
Max.

```cpp
template <typename T> std::vector<T> applyPermutation(const std::vector<T>& input, const std::vector<size_t>& perm) { // TODO figure out if there's a copy to be avoided here if (perm.size() == 0) { return input;
```
Apply permutation.

```cpp
std::vector<T> result(perm.size());
```
Result.

```cpp
for (size_t i = 0; i < perm.size(); i++) { result[i] = input[perm[i]];
```
For.

```cpp
template <typename T> std::vector<T> gather(const std::vector<T>& input, const std::vector<uint32_t>& perm) { // TODO figure out if there's a copy to be avoided here if (perm.size() == 0) { return input;
```
Gather.

```cpp
std::vector<T> result(perm.size());
```
Result.

```cpp
for (size_t i = 0; i < perm.size(); i++) { result[i] = input[perm[i]];
```
For.

```cpp
inline double randomUnit() { std::uniform_real_distribution<double> dist(0., 1.);
```
Random unit.

```cpp
return dist(util_mersenne_twister);
```
Dist.

```cpp
inline double randomReal(double minVal, double maxVal) { std::uniform_real_distribution<double> dist(minVal, maxVal);
```
Random real.

```cpp
return dist(util_mersenne_twister);
```
Dist.

```cpp
inline int randomInt(int lower, int upper) { std::uniform_int_distribution<int> dist(lower, upper);
```
Random int.

```cpp
return dist(util_mersenne_twister);
```
Dist.

```cpp
inline size_t randomIndex(size_t size) { std::uniform_int_distribution<size_t> dist(0, size - 1);
```
Random index.

```cpp
return dist(util_mersenne_twister);
```
Dist.

```cpp
inline double randomNormal(double mean = 0.0, double stddev = 1.0) { std::normal_distribution<double> dist{mean, stddev};
```
Random normal.

```cpp
return dist(util_mersenne_twister);
```
Dist.

```cpp
void ImGuiHelperMarker(const char* text);
```
Im gui helper marker.

## vector_quantity.h

```cpp
template <typename QuantityT> class VectorQuantityBase {
```
Class VectorQuantityBase.

```cpp
VectorQuantityBase(QuantityT& parent, VectorType vectorType);
```
Vector quantity base.

```cpp
void buildVectorUI();
```
Build vector ui.

```cpp
QuantityT& quantity;
```
Member quantity.

```cpp
QuantityT* setVectorLengthScale(double newLength, bool isRelative = true);
```
Set vector length scale.

```cpp
double getVectorLengthScale();
```
Get vector length scale.

```cpp
QuantityT* setVectorLengthRange(double newLength);
```
Set vector length range.

```cpp
double getVectorLengthRange();
```
Get vector length range.

```cpp
QuantityT* setVectorRadius(double val, bool isRelative = true);
```
Set vector radius.

```cpp
double getVectorRadius();
```
Get vector radius.

```cpp
QuantityT* setVectorColor(glm::vec3 color);
```
Set vector color.

```cpp
glm::vec3 getVectorColor();
```
Get vector color.

```cpp
QuantityT* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
const VectorType vectorType;
```
Member vector type.

```cpp
PersistentValue<ScaledValue<float>> vectorLengthMult;
```
Member vector length mult.

```cpp
PersistentValue<ScaledValue<float>> vectorRadius;
```
Member vector radius.

```cpp
PersistentValue<glm::vec3> vectorColor;
```
Member vector color.

```cpp
PersistentValue<std::string> material;
```
Member material.

```cpp
bool vectorLengthRangeManuallySet = false;
```
Member false.

```cpp
std::shared_ptr<render::ShaderProgram> vectorProgram;
```
Member vector program.

```cpp
template <typename QuantityT> class VectorQuantity : public VectorQuantityBase<QuantityT> {
```
Class VectorQuantity.

```cpp
VectorQuantity(QuantityT& parent, const std::vector<glm::vec3>& vectors, render::ManagedBuffer<glm::vec3>& vectorRoots, VectorType vectorType);
```
Vector quantity.

```cpp
void drawVectors();
```
Draw vectors.

```cpp
void refreshVectors();
```
Refresh vectors.

```cpp
template <class V> void updateData(const V& newVectors);
```
Update data.

```cpp
template <class V> void updateData2D(const V& newVectors);
```
Update data 2 d.

```cpp
render::ManagedBuffer<glm::vec3> vectors;
```
Member vectors.

```cpp
render::ManagedBuffer<glm::vec3>& vectorRoots;
```
Member vector roots.

```cpp
void createProgram();
```
Create program.

```cpp
void updateMaxLength();
```
Update max length.

```cpp
std::vector<glm::vec3> vectorsData;
```
Member vectors data.

```cpp
template <typename QuantityT> class TangentVectorQuantity : public VectorQuantityBase<QuantityT> {
```
Class TangentVectorQuantity.

```cpp
TangentVectorQuantity(QuantityT& parent, const std::vector<glm::vec2>& tangentVectors, const std::vector<glm::vec3>& tangentBasisX, const std::vector<glm::vec3>& tangentBasisY, render::ManagedBuffer<glm::vec3>& vectorRoots, int nSym, VectorType vectorType);
```
Tangent vector quantity.

```cpp
void drawVectors();
```
Draw vectors.

```cpp
void refreshVectors();
```
Refresh vectors.

```cpp
template <class V> void updateData(const V& newVectors);
```
Update data.

```cpp
render::ManagedBuffer<glm::vec2> tangentVectors;
```
Member tangent vectors.

```cpp
render::ManagedBuffer<glm::vec3> tangentBasisX;
```
Member tangent basis x.

```cpp
render::ManagedBuffer<glm::vec3> tangentBasisY;
```
Member tangent basis y.

```cpp
render::ManagedBuffer<glm::vec3>& vectorRoots;
```
Member vector roots.

```cpp
void createProgram();
```
Create program.

```cpp
void updateMaxLength();
```
Update max length.

```cpp
std::vector<glm::vec2> tangentVectorsData;
```
Member tangent vectors data.

```cpp
std::vector<glm::vec3> tangentBasisXData;
```
Member tangent basis x data.

```cpp
std::vector<glm::vec3> tangentBasisYData;
```
Member tangent basis y data.

```cpp
int nSym;
```
Member n sym.

## view.h

```cpp
CameraParameters getCameraParametersForCurrentView(); // contains all of this info // (these friendly helpers to get the same info as ^^^) glm::mat4 getCameraViewMatrix();
```
Get camera parameters for current view.

```cpp
void setCameraViewMatrix(glm::mat4 newMat);
```
Set camera view matrix.

```cpp
glm::mat4 getCameraPerspectiveMatrix();
```
Get camera perspective matrix.

```cpp
glm::vec3 getCameraWorldPosition();
```
Get camera world position.

```cpp
void getCameraFrame(glm::vec3& lookDir, glm::vec3& upDir, glm::vec3& rightDir);
```
Get camera frame.

```cpp
glm::vec3 getUpVec();
```
Get up vec.

```cpp
glm::vec3 getFrontVec();
```
Get front vec.

```cpp
void setViewToCamera(const CameraParameters& p);
```
Set view to camera.

```cpp
void lookAt(glm::vec3 cameraLocation, glm::vec3 target, bool flyTo = false);
```
Look at.

```cpp
void lookAt(glm::vec3 cameraLocation, glm::vec3 target, glm::vec3 upDir, bool flyTo = false);
```
Look at.

```cpp
glm::mat4 computeHomeView();
```
Compute home view.

```cpp
void resetCameraToHomeView();
```
Reset camera to home view.

```cpp
void flyToHomeView();
```
Fly to home view.

```cpp
void setViewCenter(glm::vec3 newCenter, bool flyTo = false);
```
Set view center.

```cpp
glm::vec3 getViewCenter();
```
Get view center.

```cpp
void updateViewAndChangeNavigationStyle(NavigateStyle newStyle, bool flyTo = false);
```
Update view and change navigation style.

```cpp
void updateViewAndChangeUpDir(UpDir newUpDir, bool flyTo = false);
```
Update view and change up dir.

```cpp
void updateViewAndChangeFrontDir(FrontDir newFrontDir, bool flyTo = false);
```
Update view and change front dir.

```cpp
void updateViewAndChangeCenter(glm::vec3 newCenter, bool flyTo = false);
```
Update view and change center.

```cpp
void startFlightTo(const CameraParameters& p, float flightLengthInSeconds = .4);
```
Start flight to.

```cpp
void startFlightTo(const glm::mat4& T, float targetFov, float flightLengthInSeconds = .4);
```
Start flight to.

```cpp
void immediatelyEndFlight();
```
Immediately end flight.

```cpp
void setWindowSize(int width, int height);
```
Set window size.

```cpp
std::tuple<int, int> getWindowSize();
```
Get window size.

```cpp
std::tuple<int, int> getBufferSize();
```
Get buffer size.

```cpp
void setUpDir(UpDir newUpDir, bool animateFlight = false);
```
Set up dir.

```cpp
UpDir getUpDir();
```
Get up dir.

```cpp
void setFrontDir(FrontDir newFrontDir, bool animateFlight = false);
```
Set front dir.

```cpp
FrontDir getFrontDir();
```
Get front dir.

```cpp
void setNavigateStyle(NavigateStyle newNavigateStyle, bool animateFlight = false);
```
Set navigate style.

```cpp
NavigateStyle getNavigateStyle();
```
Get navigate style.

```cpp
void setWindowResizable(bool isResizable);
```
Set window resizable.

```cpp
bool getWindowResizable();
```
Get window resizable.

```cpp
glm::vec3 screenCoordsToWorldRay(glm::vec2 screenCoords);
```
Screen coords to world ray.

```cpp
glm::vec3 bufferIndsToWorldRay(glm::ivec2 bufferInds);
```
Buffer inds to world ray.

```cpp
glm::vec3 screenCoordsAndDepthToWorldPosition(glm::vec2 screenCoords, float clipDepth);
```
Screen coords and depth to world position.

```cpp
std::string getViewAsJson();
```
Get view as json.

```cpp
void setViewFromJson(std::string jsonData, bool flyTo);
```
Set view from json.

```cpp
std::string getCameraJson(); // DEPRACTED: old names for avove void setCameraFromJson(std::string jsonData, bool flyTo);
```
Get camera json.

```cpp
std::string to_string(ProjectionMode mode);
```
To string.

```cpp
std::string to_string(NavigateStyle style);
```
To string.

```cpp
std::tuple<int, int> screenCoordsToBufferInds(glm::vec2 screenCoords);
```
Screen coords to buffer inds.

```cpp
glm::ivec2 screenCoordsToBufferIndsVec(glm::vec2 screenCoords);
```
Screen coords to buffer inds vec.

```cpp
glm::vec2 bufferIndsToScreenCoords(int xPos, int yPos);
```
Buffer inds to screen coords.

```cpp
glm::vec2 bufferIndsToScreenCoords(glm::ivec2 bufferInds);
```
Buffer inds to screen coords.

```cpp
void buildViewGui();
```
Build view gui.

```cpp
void updateFlight();
```
Update flight.

```cpp
void invalidateView();
```
Invalidate view.

```cpp
void ensureViewValid();
```
Ensure view valid.

```cpp
void processTranslate(glm::vec2 delta);
```
Process translate.

```cpp
void processRotate(glm::vec2 startP, glm::vec2 endP);
```
Process rotate.

```cpp
void processClipPlaneShift(double amount);
```
Process clip plane shift.

```cpp
void processZoom(double amount, bool relativeToCenter = false);
```
Process zoom.

```cpp
void processKeyboardNavigation(ImGuiIO& io);
```
Process keyboard navigation.

```cpp
void processSetCenter(glm::vec2 screenCoords);
```
Process set center.

```cpp
glm::vec3 bufferCoordsToWorldRay(glm::vec2 bufferCoords);
```
Buffer coords to world ray.

## volume_grid.h

```cpp
class VolumeGrid;
```
Class VolumeGrid.

```cpp
class VolumeGridNodeScalarQuantity;
```
Class VolumeGridNodeScalarQuantity.

```cpp
class VolumeGridCellScalarQuantity;
```
Class VolumeGridCellScalarQuantity.

```cpp
template <> // Specialize the quantity type struct QuantityTypeHelper<VolumeGrid> {
```
Struct QuantityTypeHelper.

```cpp
struct VolumeGridPickResult {
```
Struct VolumeGridPickResult.

```cpp
VolumeGridElement elementType;
```
Member element type.

```cpp
int64_t index;
```
Member index.

```cpp
class VolumeGrid : public QuantityStructure<VolumeGrid> {
```
Class VolumeGrid.

```cpp
VolumeGrid(std::string name, glm::uvec3 gridNodeDim_, glm::vec3 boundMin_, glm::vec3 boundMax_);
```
Volume grid.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void buildPickUI(const PickResult& result) override;
```
Build pick ui.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
render::ManagedBuffer<glm::vec3> gridPlaneReferencePositions;
```
Member grid plane reference positions.

```cpp
render::ManagedBuffer<glm::vec3> gridPlaneReferenceNormals;
```
Member grid plane reference normals.

```cpp
render::ManagedBuffer<int32_t> gridPlaneAxisInds;
```
Member grid plane axis inds.

```cpp
template <class T> VolumeGridNodeScalarQuantity* addNodeScalarQuantity(std::string name, const T& values, DataType dataType_ = DataType::STANDARD);
```
Add node scalar quantity.

```cpp
template <class Func> VolumeGridNodeScalarQuantity* addNodeScalarQuantityFromCallable(std::string name, Func&& func, DataType dataType_ = DataType::STANDARD);
```
Add node scalar quantity from callable.

```cpp
template <class Func> VolumeGridNodeScalarQuantity* addNodeScalarQuantityFromBatchCallable(std::string name, Func&& func, DataType dataType_ = DataType::STANDARD);
```
Add node scalar quantity from batch callable.

```cpp
template <class T> VolumeGridCellScalarQuantity* addCellScalarQuantity(std::string name, const T& values, DataType dataType_ = DataType::STANDARD);
```
Add cell scalar quantity.

```cpp
template <class Func> VolumeGridCellScalarQuantity* addCellScalarQuantityFromCallable(std::string name, Func&& func, DataType dataType_ = DataType::STANDARD);
```
Add cell scalar quantity from callable.

```cpp
template <class Func> VolumeGridCellScalarQuantity* addCellScalarQuantityFromBatchCallable(std::string name, Func&& func, DataType dataType_ = DataType::STANDARD);
```
Add cell scalar quantity from batch callable.

```cpp
std::vector<std::string> addGridCubeRules(std::vector<std::string> initRules, bool withShade=true);
```
Add grid cube rules.

```cpp
void setVolumeGridUniforms(render::ShaderProgram& p);
```
Set volume grid uniforms.

```cpp
void setGridCubeUniforms(render::ShaderProgram& p, bool withShade=true);
```
Set grid cube uniforms.

```cpp
uint64_t nNodes() const;
```
N nodes.

```cpp
uint64_t nCells() const;
```
N cells.

```cpp
glm::vec3 gridSpacing() const;
```
Grid spacing.

```cpp
glm::vec3 gridSpacingReference() const;
```
Grid spacing reference.

```cpp
float minGridSpacing() const;
```
Min grid spacing.

```cpp
glm::uvec3 getGridNodeDim() const;
```
Get grid node dim.

```cpp
glm::uvec3 getGridCellDim() const;
```
Get grid cell dim.

```cpp
glm::vec3 getBoundMin() const;
```
Get bound min.

```cpp
glm::vec3 getBoundMax() const;
```
Get bound max.

```cpp
uint64_t flattenNodeIndex(glm::uvec3 inds) const;
```
Flatten node index.

```cpp
glm::uvec3 unflattenNodeIndex(uint64_t i) const;
```
Unflatten node index.

```cpp
glm::vec3 positionOfNodeIndex(uint64_t i) const;
```
Position of node index.

```cpp
glm::vec3 positionOfNodeIndex(glm::uvec3 inds) const;
```
Position of node index.

```cpp
uint64_t flattenCellIndex(glm::uvec3 inds) const;
```
Flatten cell index.

```cpp
glm::uvec3 unflattenCellIndex(uint64_t i) const;
```
Unflatten cell index.

```cpp
glm::vec3 positionOfCellIndex(uint64_t i) const;
```
Position of cell index.

```cpp
glm::vec3 positionOfCellIndex(glm::uvec3 inds) const;
```
Position of cell index.

```cpp
void markNodesAsUsed();
```
Mark nodes as used.

```cpp
void markCellsAsUsed();
```
Mark cells as used.

```cpp
VolumeGridPickResult interpretPickResult(const PickResult& result);
```
Interpret pick result.

```cpp
VolumeGrid* setColor(glm::vec3 val);
```
Set color.

```cpp
glm::vec3 getColor();
```
Get color.

```cpp
VolumeGrid* setEdgeColor(glm::vec3 val);
```
Set edge color.

```cpp
glm::vec3 getEdgeColor();
```
Get edge color.

```cpp
VolumeGrid* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
VolumeGrid* setEdgeWidth(double newVal);
```
Set edge width.

```cpp
double getEdgeWidth();
```
Get edge width.

```cpp
VolumeGrid* setCubeSizeFactor(double newVal);
```
Set cube size factor.

```cpp
double getCubeSizeFactor();
```
Get cube size factor.

```cpp
glm::uvec3 gridNodeDim;
```
Member grid node dim.

```cpp
glm::uvec3 gridCellDim;
```
Member grid cell dim.

```cpp
glm::vec3 boundMin, boundMax;
```
Member bound min.

```cpp
glm::vec3 boundMin, boundMax;
```
Member bound max.

```cpp
std::vector<glm::vec3> gridPlaneReferencePositionsData;
```
Member grid plane reference positions data.

```cpp
std::vector<glm::vec3> gridPlaneReferenceNormalsData;
```
Member grid plane reference normals data.

```cpp
std::vector<int32_t> gridPlaneAxisIndsData;
```
Member grid plane axis inds data.

```cpp
PersistentValue<glm::vec3> color;
```
Member color.

```cpp
PersistentValue<glm::vec3> edgeColor;
```
Member edge color.

```cpp
PersistentValue<std::string> material;
```
Member material.

```cpp
PersistentValue<float> edgeWidth;
```
Member edge width.

```cpp
PersistentValue<float> cubeSizeFactor;
```
Member cube size factor.

```cpp
void computeGridPlaneReferenceGeometry();
```
Compute grid plane reference geometry.

```cpp
size_t globalPickConstant = INVALID_IND_64;
```
Member invalid ind 64.

```cpp
glm::vec3 pickColor;
```
Member pick color.

```cpp
void buildNodeInfoGUI(const VolumeGridPickResult& result);
```
Build node info gui.

```cpp
void buildCellInfoGUI(const VolumeGridPickResult& result);
```
Build cell info gui.

```cpp
bool nodesHaveBeenUsed = false;
```
Member false.

```cpp
bool cellsHaveBeenUsed = false;
```
Member false.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
std::shared_ptr<render::ShaderProgram> pickProgram;
```
Member pick program.

```cpp
void ensureGridCubeRenderProgramPrepared();
```
Ensure grid cube render program prepared.

```cpp
void ensureGridCubePickProgramPrepared();
```
Ensure grid cube pick program prepared.

```cpp
VolumeGridNodeScalarQuantity* addNodeScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType dataType_);
```
Add node scalar quantity impl.

```cpp
VolumeGridCellScalarQuantity* addCellScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType dataType_);
```
Add cell scalar quantity impl.

```cpp
VolumeGrid* registerVolumeGrid(std::string name, glm::uvec3 gridNodeDim, glm::vec3 boundMin, glm::vec3 boundMax);
```
Register volume grid.

```cpp
VolumeGrid* registerVolumeGrid(std::string name, uint64_t gridNodeAxesDim, glm::vec3 boundMin, glm::vec3 boundMax);
```
Register volume grid.

```cpp
inline VolumeGrid* getVolumeGrid(std::string name = "");
```
Get volume grid.

```cpp
inline bool hasVolumeGrid(std::string name = "");
```
Return whether it has volume grid.

```cpp
inline void removeVolumeGrid(std::string name = "", bool errorIfAbsent = false);
```
Remove volume grid.

## volume_grid_quantity.h

```cpp
class VolumeGrid;
```
Forward declare structure

```cpp
class VolumeGridQuantity : public QuantityS<VolumeGrid> {
```
Extend Quantity<VolumeGrid> to add a few extra functions

```cpp
VolumeGridQuantity(std::string name, VolumeGrid& parentStructure, bool dominates = false);
```
Volume grid quantity.

```cpp
~VolumeGridQuantity() {};
```
Volume grid quantity.

```cpp
virtual bool isDrawingGridcubes() = 0;
```
Return whether drawing gridcubes.

```cpp
virtual void buildNodeInfoGUI(size_t vInd);
```
Build node info gui.

```cpp
virtual void buildCellInfoGUI(size_t vInd);
```
Build cell info gui.

## volume_grid_scalar_quantity.h

```cpp
class VolumeGridNodeScalarQuantity : public VolumeGridQuantity, public ScalarQuantity<VolumeGridNodeScalarQuantity> {
```
======================================================== ========== Node Scalar ========== ========================================================

```cpp
VolumeGridNodeScalarQuantity(std::string name, VolumeGrid& grid_, const std::vector<float>& values_, DataType dataType_);
```
Volume grid node scalar quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual void buildNodeInfoGUI(size_t ind) override;
```
Build node info gui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual bool isDrawingGridcubes() override;
```
Return whether drawing gridcubes.

```cpp
VolumeGridNodeScalarQuantity* setGridcubeVizEnabled(bool val);
```
Set gridcube viz enabled.

```cpp
bool getGridcubeVizEnabled();
```
Get gridcube viz enabled.

```cpp
VolumeGridNodeScalarQuantity* setIsosurfaceVizEnabled(bool val);
```
Set isosurface viz enabled.

```cpp
bool getIsosurfaceVizEnabled();
```
Get isosurface viz enabled.

```cpp
VolumeGridNodeScalarQuantity* setIsosurfaceLevel(float value);
```
Set isosurface level.

```cpp
float getIsosurfaceLevel();
```
Get isosurface level.

```cpp
VolumeGridNodeScalarQuantity* setIsosurfaceColor(glm::vec3 val);
```
Set isosurface color.

```cpp
glm::vec3 getIsosurfaceColor();
```
Get isosurface color.

```cpp
VolumeGridNodeScalarQuantity* setSlicePlanesAffectIsosurface(bool val);
```
Set slice planes affect isosurface.

```cpp
bool getSlicePlanesAffectIsosurface();
```
Get slice planes affect isosurface.

```cpp
SurfaceMesh* registerIsosurfaceAsMesh(std::string structureName = "");
```
Register isosurface as mesh.

```cpp
PersistentValue<bool> gridcubeVizEnabled;
```
Member gridcube viz enabled.

```cpp
std::shared_ptr<render::ShaderProgram> gridcubeProgram;
```
Member gridcube program.

```cpp
void createGridcubeProgram();
```
Create gridcube program.

```cpp
PersistentValue<bool> isosurfaceVizEnabled;
```
Member isosurface viz enabled.

```cpp
PersistentValue<float> isosurfaceLevel;
```
Member isosurface level.

```cpp
PersistentValue<glm::vec3> isosurfaceColor;
```
Member isosurface color.

```cpp
PersistentValue<bool> slicePlanesAffectIsosurface;
```
Member slice planes affect isosurface.

```cpp
std::shared_ptr<render::ShaderProgram> isosurfaceProgram;
```
Member isosurface program.

```cpp
void createIsosurfaceProgram();
```
Create isosurface program.

```cpp
class VolumeGridCellScalarQuantity : public VolumeGridQuantity, public ScalarQuantity<VolumeGridCellScalarQuantity> {
```
======================================================== ========== Cell Scalar ========== ========================================================

```cpp
VolumeGridCellScalarQuantity(std::string name, VolumeGrid& grid_, const std::vector<float>& values_, DataType dataType_);
```
Volume grid cell scalar quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual void buildCellInfoGUI(size_t ind) override;
```
Build cell info gui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual bool isDrawingGridcubes() override;
```
Return whether drawing gridcubes.

```cpp
VolumeGridCellScalarQuantity* setGridcubeVizEnabled(bool val);
```
Set gridcube viz enabled.

```cpp
bool getGridcubeVizEnabled();
```
Get gridcube viz enabled.

```cpp
PersistentValue<bool> gridcubeVizEnabled;
```
Member gridcube viz enabled.

```cpp
std::shared_ptr<render::ShaderProgram> gridcubeProgram;
```
Member gridcube program.

```cpp
void createGridcubeProgram();
```
Create gridcube program.

## volume_mesh.h

```cpp
class VolumeMeshVertexColorQuantity;
```
Forward declarations for quantities

```cpp
class VolumeMeshCellColorQuantity;
```
Class VolumeMeshCellColorQuantity.

```cpp
class VolumeMeshVertexScalarQuantity;
```
Class VolumeMeshVertexScalarQuantity.

```cpp
class VolumeMeshCellScalarQuantity;
```
Class VolumeMeshCellScalarQuantity.

```cpp
class VolumeMeshVertexVectorQuantity;
```
Class VolumeMeshVertexVectorQuantity.

```cpp
class VolumeMeshCellVectorQuantity;
```
Class VolumeMeshCellVectorQuantity.

```cpp
template <> // Specialize the quantity type struct QuantityTypeHelper<VolumeMesh> {
```
Struct QuantityTypeHelper.

```cpp
struct VolumeMeshPickResult {
```
Struct VolumeMeshPickResult.

```cpp
VolumeMeshElement elementType;
```
Member element type.

```cpp
int64_t index;
```
Member index.

```cpp
class VolumeMesh : public QuantityStructure<VolumeMesh> {
```
=== The grand volume mesh class

```cpp
VolumeMesh(std::string name, const std::vector<glm::vec3>& vertexPositions, const std::vector<std::array<uint32_t, 8>>& cellIndices);
```
Volume mesh.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildCustomOptionsUI() override;
```
Build custom options ui.

```cpp
virtual void buildPickUI(const PickResult& result) override;
```
Build pick ui.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawDelayed() override;
```
Draw delayed.

```cpp
virtual void drawPick() override;
```
Draw pick.

```cpp
virtual void updateObjectSpaceBounds() override;
```
Update object space bounds.

```cpp
virtual std::string typeName() override;
```
Type name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
render::ManagedBuffer<glm::vec3> vertexPositions;
```
Member vertex positions.

```cpp
render::ManagedBuffer<uint32_t> triangleVertexInds;
```
Member triangle vertex inds.

```cpp
render::ManagedBuffer<uint32_t> triangleFaceInds;
```
Member triangle face inds.

```cpp
render::ManagedBuffer<uint32_t> triangleCellInds;
```
Member triangle cell inds.

```cpp
render::ManagedBuffer<glm::vec3> baryCoord;
```
Member bary coord.

```cpp
render::ManagedBuffer<glm::vec3> edgeIsReal;
```
Member edge is real.

```cpp
render::ManagedBuffer<float> faceType;
```
Member face type.

```cpp
render::ManagedBuffer<glm::vec3> faceNormals;
```
Member face normals.

```cpp
render::ManagedBuffer<glm::vec3> cellCenters;
```
Member cell centers.

```cpp
template <class T> VolumeMeshVertexScalarQuantity* addVertexScalarQuantity(std::string name, const T& data, DataType type = DataType::STANDARD);
```
Add vertex scalar quantity.

```cpp
template <class T> VolumeMeshCellScalarQuantity* addCellScalarQuantity(std::string name, const T& data, DataType type = DataType::STANDARD);
```
Add cell scalar quantity.

```cpp
template <class T> VolumeMeshVertexColorQuantity* addVertexColorQuantity(std::string name, const T& data);
```
Add vertex color quantity.

```cpp
template <class T> VolumeMeshCellColorQuantity* addCellColorQuantity(std::string name, const T& data);
```
Add cell color quantity.

```cpp
template <class T> VolumeMeshVertexVectorQuantity* addVertexVectorQuantity(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add vertex vector quantity.

```cpp
template <class T> VolumeMeshCellVectorQuantity* addCellVectorQuantity(std::string name, const T& vectors, VectorType vectorType = VectorType::STANDARD);
```
Add cell vector quantity.

```cpp
template <class V> void updateVertexPositions(const V& newPositions);
```
Update vertex positions.

```cpp
std::vector<std::array<uint32_t, 8>> cells;
```
Member uint 32 t.

```cpp
std::vector<std::array<uint32_t, 8>> cells;
```
Member cells.

```cpp
size_t nVertices() { return vertexPositions.size(); }
```
N vertices.

```cpp
size_t nCells() { return cells.size(); }
```
N cells.

```cpp
size_t nFacesTriangulation() const { return nFacesTriangulationCount; }
```
N faces triangulation.

```cpp
size_t nFaces() const { return nFacesCount; }
```
N faces.

```cpp
std::vector<char> faceIsInterior;
```
Member face is interior.

```cpp
VolumeCellType cellType(size_t i) const;
```
Cell type.

```cpp
void computeCounts();
```
Compute counts.

```cpp
void computeConnectivityData();
```
Compute connectivity data.

```cpp
std::vector<std::string> addVolumeMeshRules(std::vector<std::string> initRules, bool withSurfaceShade = true, bool isSlice = false);
```
Add volume mesh rules.

```cpp
std::vector<std::array<uint32_t, 4>> tets;
```
Member uint 32 t.

```cpp
std::vector<std::array<uint32_t, 4>> tets;
```
Member tets.

```cpp
size_t nTets();
```
N tets.

```cpp
void computeTets();
```
Compute tets.

```cpp
void ensureHaveTets();
```
Ensure have tets.

```cpp
VolumeMeshPickResult interpretPickResult(const PickResult& result);
```
Interpret pick result.

```cpp
static const std::string structureTypeName;
```
Member structure type name.

```cpp
VolumeMesh* setColor(glm::vec3 val);
```
Set color.

```cpp
glm::vec3 getColor();
```
Get color.

```cpp
VolumeMesh* setInteriorColor(glm::vec3 val);
```
Set interior color.

```cpp
glm::vec3 getInteriorColor();
```
Get interior color.

```cpp
VolumeMesh* setEdgeColor(glm::vec3 val);
```
Set edge color.

```cpp
glm::vec3 getEdgeColor();
```
Get edge color.

```cpp
VolumeMesh* setMaterial(std::string name);
```
Set material.

```cpp
std::string getMaterial();
```
Get material.

```cpp
VolumeMesh* setEdgeWidth(double newVal);
```
Set edge width.

```cpp
double getEdgeWidth();
```
Get edge width.

```cpp
VolumeMeshVertexScalarQuantity* getLevelSetQuantity();
```
Get level set quantity.

```cpp
void setLevelSetQuantity(VolumeMeshVertexScalarQuantity* _levelSet);
```
Set level set quantity.

```cpp
void setVolumeMeshUniforms(render::ShaderProgram& p);
```
Set volume mesh uniforms.

```cpp
void fillGeometryBuffers(render::ShaderProgram& p);
```
Fill geometry buffers.

```cpp
void fillSliceGeometryBuffers(render::ShaderProgram& p);
```
Fill slice geometry buffers.

```cpp
static const std::vector<std::vector<std::array<size_t, 3>>>& cellStencil(VolumeCellType type);
```
Cell stencil.

```cpp
std::vector<polyscope::SlicePlane*> volumeSlicePlaneListeners;
```
Member volume slice plane listeners.

```cpp
void addSlicePlaneListener(polyscope::SlicePlane* sp);
```
Add slice plane listener.

```cpp
void removeSlicePlaneListener(polyscope::SlicePlane* sp);
```
Remove slice plane listener.

```cpp
void refreshVolumeMeshListeners();
```
Refresh volume mesh listeners.

```cpp
std::vector<glm::vec3> vertexPositionsData;
```
Member vertex positions data.

```cpp
std::vector<uint32_t> triangleVertexIndsData;
```
Member triangle vertex inds data.

```cpp
std::vector<uint32_t> triangleFaceIndsData;
```
Member triangle face inds data.

```cpp
std::vector<uint32_t> triangleCellIndsData;
```
Member triangle cell inds data.

```cpp
std::vector<glm::vec3> baryCoordData;
```
Member bary coord data.

```cpp
std::vector<glm::vec3> edgeIsRealData;
```
Member edge is real data.

```cpp
std::vector<float> faceTypeData;
```
Member face type data.

```cpp
std::vector<glm::vec3> faceNormalsData;
```
Member face normals data.

```cpp
std::vector<glm::vec3> cellCentersData;
```
Member cell centers data.

```cpp
PersistentValue<glm::vec3> color;
```
Member color.

```cpp
PersistentValue<glm::vec3> interiorColor;
```
Member interior color.

```cpp
PersistentValue<glm::vec3> edgeColor;
```
Member edge color.

```cpp
PersistentValue<std::string> material;
```
Member material.

```cpp
PersistentValue<float> edgeWidth;
```
Member edge width.

```cpp
float activeLevelSetValue;
```
Member active level set value.

```cpp
VolumeMeshVertexScalarQuantity* activeLevelSetQuantity;
```
Member active level set quantity.

```cpp
void prepare();
```
Prepare.

```cpp
void preparePick();
```
Prepare pick.

```cpp
void geometryChanged();
```
Geometry changed.

```cpp
void recomputeGeometryIfPopulated();
```
Recompute geometry if populated.

```cpp
size_t cellPickIndStart;
```
Member cell pick ind start.

```cpp
void buildVertexInfoGui(size_t vInd);
```
Build vertex info gui.

```cpp
void buildCellInfoGUI(size_t cInd);
```
Build cell info gui.

```cpp
void computeFaceNormals();
```
Compute face normals.

```cpp
void computeCellCenters();
```
Compute cell centers.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
std::shared_ptr<render::ShaderProgram> pickProgram;
```
Member pick program.

```cpp
void initializeMeshTriangulation();
```
Initialize mesh triangulation.

```cpp
void fillGeometryBuffersFlat(render::ShaderProgram& p);
```
Fill geometry buffers flat.

```cpp
static const std::vector<std::vector<std::array<size_t, 3>>> stencilTet;
```
Member size t.

```cpp
static const std::vector<std::vector<std::array<size_t, 3>>> stencilTet;
```
Member stencil tet.

```cpp
static const std::vector<std::vector<std::array<size_t, 3>>> stencilHex;
```
Member size t.

```cpp
static const std::vector<std::vector<std::array<size_t, 3>>> stencilHex;
```
Member stencil hex.

```cpp
static const std::array<std::array<size_t, 8>, 8> rotationMap;
```
Member size t.

```cpp
static const std::array<std::array<size_t, 8>, 8> rotationMap;
```
Member rotation map.

```cpp
static const std::array<std::array<std::array<size_t, 4>, 6>, 4> diagonalMap;
```
Member size t.

```cpp
static const std::array<std::array<std::array<size_t, 4>, 6>, 4> diagonalMap;
```
Member diagonal map.

```cpp
VolumeMeshVertexColorQuantity* addVertexColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
```
Add vertex color quantity impl.

```cpp
VolumeMeshCellColorQuantity* addCellColorQuantityImpl(std::string name, const std::vector<glm::vec3>& colors);
```
Add cell color quantity impl.

```cpp
VolumeMeshVertexScalarQuantity* addVertexScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add vertex scalar quantity impl.

```cpp
VolumeMeshCellScalarQuantity* addCellScalarQuantityImpl(std::string name, const std::vector<float>& data, DataType type);
```
Add cell scalar quantity impl.

```cpp
VolumeMeshVertexVectorQuantity* addVertexVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors, VectorType vectorType);
```
Add vertex vector quantity impl.

```cpp
VolumeMeshCellVectorQuantity* addCellVectorQuantityImpl(std::string name, const std::vector<glm::vec3>& vectors, VectorType vectorType);
```
Add cell vector quantity impl.

```cpp
template <class V, class C> VolumeMesh* registerTetMesh(std::string name, const V& vertexPositions, const C& tetIndices);
```
Register tet mesh.

```cpp
template <class V, class C> template <class V, class C> VolumeMesh* registerHexMesh(std::string name, const V& vertexPositions, const C& hexIndices);
```
Register hex mesh.

```cpp
template <class V, class C> template <class V, class C> VolumeMesh* registerVolumeMesh(std::string name, const V& vertexPositions, const C& hexIndices);
```
Register volume mesh.

```cpp
template <class V, class C> template <class V, class Ct, class Ch> VolumeMesh* registerTetHexMesh(std::string name, const V& vertexPositions, const Ct& tetIndices, const Ch& hexIndices);
```
Register tet hex mesh.

```cpp
inline VolumeMesh* getVolumeMesh(std::string name = "");
```
Get volume mesh.

```cpp
inline bool hasVolumeMesh(std::string name = "");
```
Return whether it has volume mesh.

```cpp
inline void removeVolumeMesh(std::string name = "", bool errorIfAbsent = false);
```
Remove volume mesh.

## volume_mesh_color_quantity.h

```cpp
class VolumeMeshQuantity;
```
forward declaration

```cpp
class VolumeMesh;
```
Class VolumeMesh.

```cpp
class VolumeMeshColorQuantity : public VolumeMeshQuantity, public ColorQuantity<VolumeMeshColorQuantity> {
```
Class VolumeMeshColorQuantity.

```cpp
VolumeMeshColorQuantity(std::string name, VolumeMesh& mesh_, std::string definedOn, const std::vector<glm::vec3>& colorValues);
```
Volume mesh color quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
const std::string definedOn;
```
Member defined on.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
std::shared_ptr<render::ShaderProgram> sliceProgram;
```
Member slice program.

```cpp
virtual void createProgram() = 0;
```
Create program.

```cpp
class VolumeMeshVertexColorQuantity : public VolumeMeshColorQuantity {
```
======================================================== ========== Vertex Color ========== ========================================================

```cpp
VolumeMeshVertexColorQuantity(std::string name, VolumeMesh& mesh_, const std::vector<glm::vec3>& values_);
```
Volume mesh vertex color quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual std::shared_ptr<render::ShaderProgram> createSliceProgram() override;
```
Create slice program.

```cpp
void fillSliceColorBuffers(render::ShaderProgram& p);
```
Fill slice color buffers.

```cpp
virtual void drawSlice(polyscope::SlicePlane* sp) override;
```
Draw slice.

```cpp
void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
class VolumeMeshCellColorQuantity : public VolumeMeshColorQuantity {
```
======================================================== ========== Cell Color ========== ========================================================

```cpp
VolumeMeshCellColorQuantity(std::string name, VolumeMesh& mesh_, const std::vector<glm::vec3>& values_);
```
Volume mesh cell color quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
void buildCellInfoGUI(size_t cInd) override;
```
Build cell info gui.

## volume_mesh_quantity.h

```cpp
class VolumeMesh;
```
Forward declare volume mesh

```cpp
class VolumeMeshQuantity : public QuantityS<VolumeMesh> {
```
Extend Quantity<VolumeMesh> to add a few extra functions

```cpp
VolumeMeshQuantity(std::string name, VolumeMesh& parentStructure, bool dominates = false);
```
Volume mesh quantity.

```cpp
~VolumeMeshQuantity() {};
```
Volume mesh quantity.

```cpp
virtual std::shared_ptr<render::ShaderProgram> createSliceProgram() { return nullptr; };
```
Create slice program.

```cpp
virtual void drawSlice(polyscope::SlicePlane* sp) {};
```
Draw slice.

```cpp
virtual void buildVertexInfoGUI(size_t vInd);
```
Build vertex info gui.

```cpp
virtual void buildEdgeInfoGUI(size_t eInd);
```
Build edge info gui.

```cpp
virtual void buildFaceInfoGUI(size_t fInd);
```
Build face info gui.

```cpp
virtual void buildCellInfoGUI(size_t cInd);
```
Build cell info gui.

## volume_mesh_scalar_quantity.h

```cpp
class VolumeMeshScalarQuantity : public VolumeMeshQuantity, public ScalarQuantity<VolumeMeshScalarQuantity> {
```
Class VolumeMeshScalarQuantity.

```cpp
VolumeMeshScalarQuantity(std::string name, VolumeMesh& mesh_, std::string definedOn, const std::vector<float>& values_, DataType dataType);
```
Volume mesh scalar quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
const std::string definedOn;
```
Member defined on.

```cpp
std::shared_ptr<render::ShaderProgram> program;
```
Member program.

```cpp
std::shared_ptr<render::ShaderProgram> sliceProgram;
```
Member slice program.

```cpp
virtual void createProgram() = 0;
```
Create program.

```cpp
class VolumeMeshVertexScalarQuantity : public VolumeMeshScalarQuantity {
```
======================================================== ========== Vertex Scalar ========== ========================================================

```cpp
VolumeMeshVertexScalarQuantity(std::string name, const std::vector<float>& values_, VolumeMesh& mesh_, DataType dataType_ = DataType::STANDARD);
```
Volume mesh vertex scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
virtual std::shared_ptr<render::ShaderProgram> createSliceProgram() override;
```
Create slice program.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void drawSlice(polyscope::SlicePlane* sp) override;
```
Draw slice.

```cpp
void setLevelSetValue(float f);
```
Set level set value.

```cpp
void setEnabledLevelSet(bool v);
```
Set enabled level set.

```cpp
void setLevelSetVisibleQuantity(std::string name);
```
Set level set visible quantity.

```cpp
void setLevelSetUniforms(render::ShaderProgram& p);
```
Set level set uniforms.

```cpp
void fillLevelSetData(render::ShaderProgram& p);
```
Fill level set data.

```cpp
std::shared_ptr<render::ShaderProgram> levelSetProgram;
```
Member level set program.

```cpp
void fillSliceColorBuffers(render::ShaderProgram& p);
```
Fill slice color buffers.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void buildScalarOptionsUI() override;
```
Build scalar options ui.

```cpp
void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
float levelSetValue;
```
Member level set value.

```cpp
bool isDrawingLevelSet;
```
Member is drawing level set.

```cpp
VolumeMeshVertexScalarQuantity* showQuantity;
```
Member show quantity.

```cpp
class VolumeMeshCellScalarQuantity : public VolumeMeshScalarQuantity {
```
======================================================== ========== Cell Scalar ========== ========================================================

```cpp
VolumeMeshCellScalarQuantity(std::string name, const std::vector<float>& values_, VolumeMesh& mesh_, DataType dataType_ = DataType::STANDARD);
```
Volume mesh cell scalar quantity.

```cpp
virtual void createProgram() override;
```
Create program.

```cpp
void fillColorBuffers(render::ShaderProgram& p);
```
Fill color buffers.

```cpp
void buildCellInfoGUI(size_t fInd) override;
```
Build cell info gui.

## volume_mesh_vector_quantity.h

```cpp
class VolumeMeshVectorQuantity : public VolumeMeshQuantity {
```
==== Common base class Represents a general vector field associated with a surface mesh, including R3 fields in the ambient space and R2 fields embedded in the surface NOTE: This intermediate class is not really necessary anymore; it is subsumed by the VectorQuantity<> classes which serve as common bases for ALL vector types.

```cpp
VolumeMeshVectorQuantity(std::string name, VolumeMesh& mesh_, VolumeMeshElement definedOn_);
```
Volume mesh vector quantity.

```cpp
VolumeMeshElement definedOn;
```
Member defined on.

```cpp
class VolumeMeshVertexVectorQuantity : public VolumeMeshVectorQuantity, class VolumeMeshVertexVectorQuantity : public VolumeMeshVectorQuantity, public VectorQuantity<VolumeMeshVertexVectorQuantity> {
```
Class VolumeMeshVertexVectorQuantity.

```cpp
VolumeMeshVertexVectorQuantity(std::string name, std::vector<glm::vec3> vectors_, VolumeMesh& mesh_, VectorType vectorType_ = VectorType::STANDARD);
```
Volume mesh vertex vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void buildVertexInfoGUI(size_t vInd) override;
```
Build vertex info gui.

```cpp
class VolumeMeshCellVectorQuantity : public VolumeMeshVectorQuantity, class VolumeMeshCellVectorQuantity : public VolumeMeshVectorQuantity, public VectorQuantity<VolumeMeshCellVectorQuantity> {
```
Class VolumeMeshCellVectorQuantity.

```cpp
VolumeMeshCellVectorQuantity(std::string name, std::vector<glm::vec3> vectors_, VolumeMesh& mesh_, VectorType vectorType_ = VectorType::STANDARD);
```
Volume mesh cell vector quantity.

```cpp
virtual void draw() override;
```
Draw.

```cpp
virtual void buildCustomUI() override;
```
Build custom ui.

```cpp
virtual void refresh() override;
```
Refresh.

```cpp
virtual std::string niceName() override;
```
Nice name.

```cpp
virtual void buildCellInfoGUI(size_t cInd) override;
```
Build cell info gui.

## weak_handle.h

```cpp
struct GenericWeakHandle {
```
Struct GenericWeakHandle.

```cpp
GenericWeakHandle() {} GenericWeakHandle(std::shared_ptr<WeakHandleDummyType>& sentinel_, uint64_t uniqueID_) : sentinel(std::weak_ptr<WeakHandleDummyType>(sentinel_)), targetUniqueID(uniqueID_) {} bool isValid() const;
```
Generic weak handle.

```cpp
void reset();
```
Reset.

```cpp
uint64_t getUniqueID() const;
```
Get unique id.

```cpp
std::weak_ptr<WeakHandleDummyType> sentinel;
```
Member sentinel.

```cpp
uint64_t targetUniqueID;
```
Member target unique id.

```cpp
template <typename TargetType> struct WeakHandle : public GenericWeakHandle {
```
Struct WeakHandle.

```cpp
WeakHandle() {} WeakHandle(std::shared_ptr<WeakHandleDummyType> sentinel_, uint64_t uniqueID_, TargetType* targetPtr_) : GenericWeakHandle(sentinel_, uniqueID_), targetPtr(targetPtr_) {} TargetType& get() const { return *targetPtr; };
```
Weak handle.

```cpp
TargetType* targetPtr = nullptr;
```
Member nullptr.

```cpp
class WeakReferrable {
```
Classes can extend this class

```cpp
WeakReferrable();
```
Weak referrable.

```cpp
virtual ~WeakReferrable() = default;
```
Weak referrable.

```cpp
template <typename TargetType> WeakHandle<TargetType> getWeakHandle(TargetType* targetPtr = nullptr) { if (targetPtr) { return WeakHandle<TargetType>(weakReferrableDummyRef, weakReferableUniqueID, targetPtr);
```
Get weak handle.

```cpp
} targetPtr = dynamic_cast<TargetType*>(this);
```
Function.

```cpp
if (!targetPtr) throw std::runtime_error("[Polyscope] bad getWeakHandle() cast");
```
If.

```cpp
return WeakHandle<TargetType>(weakReferrableDummyRef, weakReferableUniqueID, targetPtr);
```
Function.

```cpp
GenericWeakHandle getGenericWeakHandle() { return GenericWeakHandle(weakReferrableDummyRef, weakReferableUniqueID); };
```
Get generic weak handle.

```cpp
std::shared_ptr<WeakHandleDummyType> weakReferrableDummyRef;
```
Member weak referrable dummy ref.

```cpp
uint64_t weakReferableUniqueID;
```
Member weak referable unique id.

## widget.h

```cpp
class Widget : public virtual WeakReferrable {
```
A base class for widgets in the scene.

```cpp
Widget();
```
Widget.

```cpp
virtual ~Widget();
```
Widget.

```cpp
Widget(const Widget&) = delete;
```
Widget.

```cpp
Widget& operator=(const Widget&) = delete;
```
Function.

```cpp
virtual void draw();
```
Draw.

```cpp
virtual bool interact();
```
Interact.

```cpp
virtual void buildGUI();
```
Build gui.

