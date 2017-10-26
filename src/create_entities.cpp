#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreEntity.h>
#include <move4d-gui/common/Robot.hpp>

#undef QT_LIBRARY
#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

Ogre::SceneNode *createOgreEntities(Ogre::SceneManager *sceneManager, MoveOgre::Robot *robot, Ogre::SceneNode *scNodeRobot)
{
    //this->setAndUpdate(*this->getInitialPosition());
    //mObjectNodes.assign(getRobotStruct()->no,0);
    std::string suffix="_vismod_obj";
    for(uint oi=0;oi<robot->getRobotStruct()->no;++oi){
        p3d_obj *o=robot->getRobotStruct()->o[oi];
        Ogre::ManualObject* obj1 = sceneManager->createManualObject();
        Ogre::SceneNode *scNode=dynamic_cast<Ogre::SceneNode*>(scNodeRobot->getChild(o->name));
        if(o->np > 0 && o->type != P3D_GHOST_OBJECT){
            bool not_empty(false);
            for(uint pi=0;pi<o->np;++pi){
                if(o->pol[pi]->TYPE != P3D_GHOST){
                    not_empty=true;
                    obj1->begin("Voxel/Default", Ogre::RenderOperation::OT_TRIANGLE_LIST);
                    {
                        poly_polyhedre *p=o->pol[pi]->poly;
                        p3d_compute_vertex_normals(p);
                        for(uint fp=1;fp<=p3d_get_nb_faces(p);++fp){
                            int a1,a2, b1,b2, c1,c2; // point indices of the original triangle ([abc]1) and the displaced one ([abc]2)
                            a1=(2*fp-1)*3-3;
                            a2=a1+3;
                            b1=(2*fp-1)*3-2;
                            b2=b1+3;
                            c1=(2*fp-1)*3-1;
                            c2=c1+3;
                            for(uint zoffset=0;zoffset<2;zoffset+=1){
                                for(uint i=1;i<=p3d_get_nb_points_in_face(p,fp);++i){
                                    double x,y,z;
                                    Ogre::ColourValue color;
                                    if(o->pol[pi]->color != P3dColorAny){
                                        double c[4];
                                        g3d_get_color_vect(o->pol[pi]->color,c);
                                        color = Ogre::ColourValue(c[0],
                                                c[1],
                                                c[2],
                                                c[3]*0.5 //alpha used only with certain materials (scene_blend alpha_blend)
                                                );

                                    }
                                    else if(o->pol[pi]->color_vect){
                                        color=Ogre::ColourValue(o->pol[pi]->color_vect[0],
                                                o->pol[pi]->color_vect[1],
                                                o->pol[pi]->color_vect[2],
                                                o->pol[pi]->color_vect[3] * 0.5 //alpha used only with certain materials (scene_blend alpha_blend)
                                                );
                                    }
                                    p3d_get_point_in_pos_in_face(p,fp,i,&x,&y,&z);
                                    move_point(o->pol[pi]->pos_rel_jnt,&x,&y,&z,1);
                                    z=z-zoffset*6.+3.;
                                    obj1->position(x,y,z);
                                    uint index=p3d_get_index_point_in_face(p,fp,i);
                                    obj1->normal(p->vertex_normals[index-1][0],p->vertex_normals[index-1][1],p->vertex_normals[index-1][2]);
                                    obj1->colour(color);
                                }
                                //obj1->triangle((fp+zoffset)*3-3,(fp+zoffset)*3-2,(fp+zoffset)*3-1); //original tri (1st iteration) + offset triangle (second loop)
                                obj1->triangle((2*fp-1)*3-3+zoffset*3,(2*fp-1)*3-2+zoffset*3,(2*fp-1)*3-1+zoffset*3); //original tri (1st iteration) + offset triangle (second loop)
                            }
                            obj1->triangle(a1,a2,b2);
                            obj1->triangle(a1,b2,b1);
                            obj1->triangle(c1,b1,b2);
                            obj1->triangle(c2,c1,b2);
                            obj1->triangle(a2,a1,c2);
                            obj1->triangle(c1,c2,a1);
                        }
                    }
                    obj1->end();
                }
            }
            if(not_empty){
                obj1->convertToMesh(std::string(o->name+suffix));
                Ogre::Entity *entity=sceneManager->createEntity(o->name+suffix,o->name+suffix);
                entity->setMaterialName("Voxel/Default");
                scNode->attachObject(entity);
            }
        }
        //mObjectNodes[oi]=scNode;
    }
    //updateNodes(mObjectNodes);
    return scNodeRobot;
}
