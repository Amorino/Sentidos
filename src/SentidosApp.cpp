/*
 Copyright (C)2010-2015 Paul Houx
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without modification, are permitted.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#include "cinder/app/App.h"
#include "cinder/ObjLoader.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CameraUi.h"
#include "cinder/TriMesh.h"
#include "cinder/Rand.h"

#include "cinder/ImageIo.h"
#include "Resources.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class SentidosApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void	writeObj();
    
    void mouseMove( MouseEvent event ) override;
    
    bool performPicking( vec3 *pickedPoint, vec3 *pickedNormal );
    void drawCube( const AxisAlignedBox &bounds, const Color &color );
    
private:
    TriMeshRef			mTriMesh;		//! The 3D mesh.
    TriMesh             m0Mesh;
    AxisAlignedBox	mObjectBounds; 	//! The object space bounding box of the mesh.
    mat4				mTransform;		//! Transformations (translate, rotate, scale) of the mesh.
    
    gl::VboMeshRef	mVboMesh;
    gl::VboMeshRef	mVboMeshRef;
    
    //! By caching a 3D model and its shader on the GPU, we can draw it faster.
    gl::BatchRef		mWireCube;
    gl::BatchRef		mWirePlane;
    gl::BatchRef		mMesh;
    gl::BatchRef		mNormals;
    
    gl::BatchRef		mPrimitiveNormalLines, mPrimitiveTangentLines;
    gl::BatchRef        mBatch;
    gl::GlslProgRef     mGlsl;
    CameraPersp			mCamera;
    CameraUi			mCamUi;
    
    ivec2				mMousePos;		//! Keep track of the mouse.
    Rand    mRandom;
};

void SentidosApp::setup()
{
    m0Mesh.read( loadResource( RES_KESTREL ) );
    
#if defined( CINDER_GL_ES )
    mGlsl = gl::GlslProg::create( loadAsset( "shader_es2.vert" ), loadAsset( "shader_es2.frag" ) );
#else
    mGlsl = gl::GlslProg::create( loadAsset( "shader.vert" ), loadAsset( "shader.frag" ) );
#endif
    
    ObjLoader loader( (DataSourceRef)loadResource( RES_SENTIDO_OBJ ) );
    mTriMesh = TriMesh::create( loader );
    
    if( ! loader.getAvailableAttribs().count( geom::NORMAL ) )
        mTriMesh->recalculateNormals();
    
    // Create the mesh.
    
    //mTriMesh = TriMesh::create(m0Mesh);
    
    //mTriMesh = TriMesh::create( geom::Teapot().subdivisions( 6 ) );
    
    // Get the object space bounding box of the model, for fast intersection testing.
    mObjectBounds = mTriMesh->calcBoundingBox();
    
    
    console() << "TriMesh: " << mTriMesh->getNumVertices() << std::endl;
    
    
    // Set up the camera.
    mCamera.lookAt( vec3( 5.0f, 10.0f, 10.0f ), vec3( 0 ) );
    mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 0.01f, 100.0f );
    mCamUi = CameraUi( &mCamera, getWindow() );
    
    // Create batches that render fast.
    auto lambertShader = gl::getStockShader( gl::ShaderDef().color().lambert() );
    auto colorShader = gl::getStockShader( gl::ShaderDef().color() );
    
    mMesh = gl::Batch::create( *mTriMesh, lambertShader );
    mWirePlane = gl::Batch::create( geom::WirePlane().size( vec2( 10 ) ).subdivisions( ivec2( 10 ) ), colorShader );
    
    mWireCube = gl::Batch::create( geom::WireCube(), colorShader );
    vector<gl::VboMesh::Layout> bufferLayout = {
        gl::VboMesh::Layout().usage( GL_DYNAMIC_DRAW ).attrib( geom::Attrib::POSITION, 3 ),
        gl::VboMesh::Layout().usage( GL_STATIC_DRAW ).attrib( geom::Attrib::TEX_COORD_0, 2 )
    };
    
    mVboMesh = mMesh->getVboMesh();
    mVboMeshRef = mMesh->getVboMesh();
    
    
//    TriMesh::Format fmt = TriMesh::Format().positions().normals().texCoords().tangents();
//    
//    TriMesh mesh( loader, fmt );
//    AxisAlignedBox bbox = mesh.calcBoundingBox();
//    
//    vec3 size = bbox.getMax() - bbox.getMin();
//    float scale = std::max( std::max( size.x, size.y ), size.z ) / 25.0f;
//    mPrimitiveNormalLines = gl::Batch::create( mesh >> geom::VertexNormalLines( scale ), gl::getStockShader( gl::ShaderDef().color() ) );
//    if( mesh.hasTangents() )
//        mPrimitiveTangentLines = gl::Batch::create( mesh >> geom::VertexNormalLines( scale, geom::TANGENT ), gl::getStockShader( gl::ShaderDef().color() ) );
//    else
//        mPrimitiveTangentLines.reset();
    
    
    
    console() << "VBO: " << mVboMesh->getNumVertices() << std::endl;
    
//    auto mappedPosAttribN = mVboMesh->mapAttrib3f( geom::Attrib::NORMAL, false );
//    auto mappedPosAttrib = mVboMesh->mapAttrib3f( geom::Attrib::POSITION, false );
//    
//    for( int i = 0; i < mVboMesh->getNumVertices(); i++ ) {
//        vec3 &pos = *mappedPosAttrib;
//        vec3 &ref = *mappedPosAttribN;
//        mappedPosAttrib->y = pos.y + (ref.y * 0.1);
//        mappedPosAttrib->x = pos.x + (ref.x * 0.1);
//        mappedPosAttrib->z = pos.z + (ref.z * 0.1);
//        //console() << "Normal: " << pos << std::endl;
//        ++mappedPosAttrib;
//        ++mappedPosAttribN;
//    }
//    
//
//    auto mappedPosAttribP = mVboMesh->mapAttrib3f( geom::Attrib::POSITION, false );
//    
//    for( int i = 0; i < mVboMesh->getNumVertices(); i++ ) {
//        vec3 &pos = *mappedPosAttribP;
//        console() << "Position: " << pos << std::endl;
//        ++mappedPosAttribP;
//    }
    
//    AxisAlignedBox bbox = m0Mesh.calcBoundingBox();
//
//    vec3 size = bbox.getMax() - bbox.getMin();
//    float scale = std::max( std::max( size.x, size.y ), size.z ) / 25.0f;
//    mPrimitiveNormalLines = gl::Batch::create( m0Mesh >> geom::VertexNormalLines( scale ), gl::getStockShader( gl::ShaderDef().color() ) );
//    if( m0Mesh.hasTangents() )
//        mPrimitiveTangentLines = gl::Batch::create( m0Mesh >> geom::VertexNormalLines( scale, geom::TANGENT ), gl::getStockShader( gl::ShaderDef().color() ) );
//    else
//        mPrimitiveTangentLines.reset();
    
}

void SentidosApp::update()
{
    
    float offset = getElapsedSeconds() * 0.001f;
    
    auto mappedPosAttribN = mVboMeshRef->mapAttrib3f( geom::Attrib::NORMAL, false );
    auto mappedPosAttrib = mVboMesh->mapAttrib3f( geom::Attrib::POSITION, false );
    
    for( int i = 0; i < mVboMesh->getNumVertices(); i++ ) {
        int rand = mRandom.randInt(1, 204);
        if (i == rand){
            vec3 &posref = *mappedPosAttrib;
            vec3 &ref = *mappedPosAttribN;
        
            float randy = mRandom.randFloat(0.00, ref.y * offset);
            float randx = mRandom.randFloat(0.00, ref.y * offset);
            float randz = mRandom.randFloat(0.00, ref.y * offset);
        
            mappedPosAttrib->y = posref.y + randy;
            mappedPosAttrib->x = posref.x + randx;
            mappedPosAttrib->z = posref.z + randz;
        }
            //console() << "Normal: " << pos << std::endl;
            ++mappedPosAttrib;
            ++mappedPosAttribN;
    }
    mappedPosAttrib.unmap();
    mappedPosAttribN.unmap();
    
    /*
    
    auto mappedPosAttrib = mVboMesh->mapAttrib3f( geom::Attrib::POSITION, false );
    auto mappedPosAttribRef = mVboMeshRef->mapAttrib3f( geom::Attrib::POSITION, false );
    for( int i = 0; i < mVboMesh->getNumVertices(); i++ ) {
        //vec3 &pos = *mappedPosAttrib;
        int rand = mRandom.randFloat(1, 204);
        if (i == rand){
            float rand = mRandom.randFloat(0.00, 0.03);
            vec3 &ref = *mappedPosAttribRef;
        
            
            float x, y, z;
            if (ref.x > 0)  x = ref.x + rand;
            else  x = ref.x - rand;
            if (ref.y > 0)  y = ref.y + rand;
            else  y = ref.y - rand;
            if (ref.z > 0)  z = ref.z + rand;
            else  z = ref.z - rand;
            
            mappedPosAttrib->y = y;
            mappedPosAttrib->x =  x;
            //mappedPosAttrib->z = z;
        }
        ++mappedPosAttrib;
        ++mappedPosAttribRef;
    }*/
    
    // Animate our mesh.
    //mTransform = mat4( 1.0f );
    //mTransform *= rotate( sin( (float) getElapsedSeconds() * 3.0f ) * 0.08f, vec3( 1, 0, 0 ) );
    //mTransform *= rotate( (float) getElapsedSeconds() * 0.1f, vec3( 0, 1, 0 ) );
    //mTransform *= rotate( sin( (float) getElapsedSeconds() * 4.3f ) * 0.09f, vec3( 0, 0, 1 ) );
}

void SentidosApp::writeObj()
{
    
    mBatch = gl::Batch::create(mVboMesh, mGlsl);
    
    fs::path filePath = getSaveFilePath();
    if( ! filePath.empty() ) {
        console() << "writing mesh to file path: " << filePath << std::endl;
        ci::writeObj(writeFile(filePath), mTriMesh);
    }
}


void SentidosApp::draw()
{
    // Gray background.
    gl::clear( Color::gray( 0.5f ) );
    
    // Set up the camera.
    gl::ScopedMatrices push;
    gl::setMatrices( mCamera );
    
    // Enable depth buffer.
    gl::ScopedDepth depth( true, true );
    
    // Draw the grid on the floor.
    {
        gl::ScopedColor color( Color::gray( 0.2f ) );
        mWirePlane->draw();
    }
    
    gl::setWireframeEnabled( ! gl::isWireframeEnabled() );

    
    gl::ScopedColor colorScope( Color( 1, 1, 0 ) );
    //mPrimitiveNormalLines->draw();
    
    // Draw the mesh.
    {
        gl::ScopedColor color( Color::white() );
        
        gl::ScopedGlslProg glslScope( gl::getStockShader( gl::ShaderDef().color().lambert() ));
        gl::ScopedModelMatrix model;
        
        gl::multModelMatrix( mTransform );
        
        gl::draw( mVboMesh );
        
        //mMesh->draw();
    }
    
    // Perform 3D picking now, so we can draw the result as a vector.
    
    
//    vec3 pickedPoint, pickedNormal;
//    if( performPicking( &pickedPoint, &pickedNormal ) ) {
//        gl::ScopedColor color( Color( 0, 1, 0 ) );
//        
//        // Draw an arrow to the picked point along its normal.
//        gl::ScopedGlslProg shader( gl::getStockShader( gl::ShaderDef().color().lambert() ) );
//        gl::drawVector( pickedPoint + pickedNormal, pickedPoint );
//    }
}

void SentidosApp::mouseMove( MouseEvent event )
{
    // Keep track of the mouse.
    mMousePos = event.getPos();
}

bool SentidosApp::performPicking( vec3 *pickedPoint, vec3 *pickedNormal )
{
    // Generate a ray from the camera into our world. Note that we have to
    // flip the vertical coordinate.
    float u = mMousePos.x / (float) getWindowWidth();
    float v = mMousePos.y / (float) getWindowHeight();
    Ray ray = mCamera.generateRay( u, 1.0f - v, mCamera.getAspectRatio() );
    
    // The coordinates of the bounding box are in object space, not world space,
    // so if the model was translated, rotated or scaled, the bounding box would not
    // reflect that. One solution would be to pass the transformation to the calcBoundingBox() function:
    AxisAlignedBox worldBoundsExact = mTriMesh->calcBoundingBox( mTransform ); // slow
    
    // But if you already have an object space bounding box, it's much faster to
    // approximate the world space bounding box like this:
    AxisAlignedBox worldBoundsApprox = mObjectBounds.transformed( mTransform ); // fast
    
    // Draw the object space bounding box in yellow. It will not animate,
    // because animation is done in world space.
    drawCube( mObjectBounds, Color( 1, 1, 0 ) );
    
    // Draw the exact bounding box in orange.
    drawCube( worldBoundsExact, Color( 1, 0.5f, 0 ) );
    
    // Draw the approximated bounding box in cyan.
    drawCube( worldBoundsApprox, Color( 0, 1, 1 ) );
    
    // Perform fast detection first - test against the bounding box itself.
    if( !worldBoundsExact.intersects( ray ) )
        return false;
    
    // Set initial distance to something far, far away.
    float result = FLT_MAX;
    
    // Traverse triangle list and find the closest intersecting triangle.
    const size_t polycount = mTriMesh->getNumTriangles();
    
    float distance = 0.0f;
    for( size_t i = 0; i < polycount; ++i ) {
        // Get a single triangle from the mesh.
        vec3 v0, v1, v2;
        mTriMesh->getNormals();
        mTriMesh->getTriangleVertices( i, &v0, &v1, &v2 );
        
        // Transform triangle to world space.
        v0 = vec3( mTransform * vec4( v0, 1.0 ) );
        v1 = vec3( mTransform * vec4( v1, 1.0 ) );
        v2 = vec3( mTransform * vec4( v2, 1.0 ) );
        
//        console() << "V00: " << v0 << std::endl;
//        console() << "V01: " << v1 << std::endl;
//        console() << "V02: " << v2 << std::endl;
        
        // Test to see if the ray intersects this triangle.
        if( ray.calcTriangleIntersection( v0, v1, v2, &distance ) ) {
            // Keep the result if it's closer than any intersection we've had so far.
            if( distance < result ) {
                result = distance;
                
                // Assuming this is the closest triangle, we'll calculate our normal
                // while we've got all the points handy.
                *pickedNormal = normalize( cross( v1 - v0, v2 - v0 ) );
            }
        }
    }
    
    // Did we have a hit?
    if( distance > 0 ) {
        // Calculate the exact position of the hit.
        *pickedPoint = ray.calcPosition( result );
        
        return true;
    }
    else
        return false;
}

void SentidosApp::drawCube( const AxisAlignedBox &bounds, const Color & color )
{
    gl::ScopedColor clr( color );
    gl::ScopedModelMatrix model;
    
    gl::multModelMatrix( glm::translate( bounds.getCenter() ) * glm::scale( bounds.getSize() ) );
    mWireCube->draw();
}

CINDER_APP( SentidosApp, RendererGl( RendererGl::Options().msaa( 8 ) ) )