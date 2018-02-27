#ifndef PORTABLES_BEST_OF_H
#define PORTABLES_BEST_OF_H

#include <functional>
#include <algorithm>
#include <iostream>

#include <yarp/os/PortablePair.h>
#include <yarp/os/impl/PortCommand.h>  // funzionera??
#include <yarp/os/Stamp.h>

#include <yarp/sig/Sound.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/IJoypadController.h>

#include <yarp/sig/Matrix.h>

#include <yarp/math/Vec2D.h>
#include <yarp/math/Quaternion.h>


yarp::os::ConstString envProducer_DataPort_name  = "/envelopeProducer/Data";
yarp::os::ConstString envProducer_SynchPort_name = "/envelopeProducer/Synch";
yarp::os::ConstString envConsumer_DataPort_name  = "/envelopeConsumer/Data";
yarp::os::ConstString envConsumer_SynchPort_name = "/envelopeConsumer/Synch";

typedef struct foo
{
    double setEnv;
    union
    {
        double write;
        double read;
    };
    double tot;

    foo() : setEnv(0), write(0), tot(0) {};

} Timings;

class PortablesBestOf
{
public:

    bool verbose;
    size_t size;

    // provo un po' di portable vari
    // IDL something??
    yarp::os::Stamp   stamp;
    yarp::sig::VectorOf<int>  vect;
    yarp::dev::MapGrid2D map;
    yarp::os::Bottle   bottle;
    yarp::os::Value val;
    yarp::sig::Matrix mat;
    yarp::sig::Sound sisi;
    yarp::os::Property prop;
    yarp::math::Quaternion quat;
    yarp::os::ManagedBytes managedBite;
    yarp::sig::ImageOf< yarp::sig::PixelRgb >  image;
    yarp::math::Vec2D<double> v2;
    yarp::os::impl::PortCommand pc;
    typedef yarp::os::PortablePair<yarp::os::Bottle, yarp::sig::VectorOf<int> >  PiPi;

    PiPi pipi;

    PortablesBestOf() : verbose(false), size(10) {   }

    typedef std::function<bool( PortablesBestOf&,  yarp::os::Portable *, yarp::os::Portable *)> comparella;
    typedef std::tuple<yarp::os::Portable *, std::string, comparella, Timings, bool> robo;


    // collector
    std::vector< robo > roba;

    void allocateStuff()
    {
        roba.push_back(std::make_tuple(&image,       "Image"      , &PortablesBestOf::compareImages           , Timings(), false));
        roba.push_back(std::make_tuple(&mat,         "Matrix"     , &PortablesBestOf::compareMatrixes         , Timings(), false));
        roba.push_back(std::make_tuple(&bottle,      "Bottle"     , &PortablesBestOf::compareBottles          , Timings(), false));
        roba.push_back(std::make_tuple(&stamp,       "Stamp"      , &PortablesBestOf::compareStamps           , Timings(), false));
        roba.push_back(std::make_tuple(&val,         "Value"      , &PortablesBestOf::compareValues           , Timings(), false));
        roba.push_back(std::make_tuple(&prop,        "Property"   , &PortablesBestOf::compareProperties       , Timings(), false));
        roba.push_back(std::make_tuple(&quat,        "Quaternion" , &PortablesBestOf::compareQuaternions      , Timings(), false));
        roba.push_back(std::make_tuple(&v2,          "Vector2D"   , &PortablesBestOf::compareVectors2D        , Timings(), false));
        roba.push_back(std::make_tuple(&map,         "Map"        , &PortablesBestOf::compareMapGrid2D        , Timings(), false));
        roba.push_back(std::make_tuple(&managedBite, "ManagBites" , &PortablesBestOf::compareBites            , Timings(), false));
        roba.push_back(std::make_tuple(&pipi,        "Pair"       , &PortablesBestOf::comparePiPi             , Timings(), false));
        roba.push_back(std::make_tuple(&vect,        "Vector"     , &PortablesBestOf::compareVectorsOfInt     , Timings(), false));
        roba.push_back(std::make_tuple(&sisi,        "Sound"      , &PortablesBestOf::compareSounds           , Timings(), false));
//         roba.push_back(std::make_tuple(&pc,          "PortCommand", &PortablesBestOf::comparePortCommands     , Timings(), false));

    }

    void zeroStuff()
    {
        vect.clear();
        image.zero();
        mat.zero();
        bottle.clear();
        prop.clear();
        quat.x() = 0;
        quat.y() = 0;
        quat.z() = 0;
        quat.w() = 0;

        v2.x = 0;
        v2.y = 0;

        pc.ch = '\0';
        pc.str = "\0";

        managedBite.clear();
        pipi.head.clear();
        pipi.body.clear();
    }

    void initializeStuff()
    {
        zeroStuff();
        if(size < 10)
            size = 10;

        for(int i=0; i<size; i++)
        {
            bottle.addInt(5);
            bottle.addDouble(2.0);
        }
        mat.resize(size, size);
        mat[2][0] = 5;

        // initialize all types with some data.
        stamp.update(5);
        vect.resize(size);
        map.setSize_in_cells(size,size);
        sisi.resize(size);
        managedBite.allocate(size);

        // set some stuff
        map.setOrigin(0,0, 0);
        v2.x = 5;
        v2.y = 2;
        prop.put("5", 2);
        sisi.setSafe(5,2,0);

        image.resize(size,size);
        image.safePixel(0,0) = yarp::sig::PixelRgb(1,2,3);
        image.safePixel(4,1) = yarp::sig::PixelRgb(4,5,6);
        pc.ch = '5';
        pc.str = "stringa";

        quat.x() = 5;
        quat.y() = 2;

        for(int i=0; i<size; i++)
        {
            vect[i] = i;
            managedBite.bytes().get()[i] = i;
        }

        pipi.body.resize(size);
        pipi.head=bottle;
        pipi.body=vect;
    }



#define altrimenti else

#define torna_no() return false
#define no torna_no()

#define torna_si() return true
#define si torna_si()

    bool ugualeUguale(yarp::os::Portable *a, yarp::os::Portable *b)
    {
        no;
    }

    template<typename T>
    bool ugualeUguale2( T *ref, T *test)
    {
        T * a = dynamic_cast<T*>(ref);
        T * b = dynamic_cast<T*>(test);
        return *a==*b;
    }

    bool compareBottles(yarp::os::Portable * ref, yarp::os::Portable *test)
    {
        if(verbose) std::cout << typeid(*ref).name() << '\n';
        yarp::os::Bottle * a = dynamic_cast<yarp::os::Bottle *>(ref);
        yarp::os::Bottle * b = dynamic_cast<yarp::os::Bottle *>(test);
        return ((*a)==(*b));
    }

    bool compareStamps(yarp::os::Portable * ref, yarp::os::Portable *test)
    {
        yarp::os::Stamp * a = dynamic_cast<yarp::os::Stamp *>(ref);
        yarp::os::Stamp * b = dynamic_cast<yarp::os::Stamp *>(test);
        if(verbose)
        {
            std::cout << typeid(*ref).name() << '\n';
            std::cout << "A count " << a->getCount() << " A time " << a->getTime() << std::endl;
            std::cout << "B count " << b->getCount() << " B time " << b->getTime() << std::endl;
        }
        return ( (a->getCount() == b->getCount()) && (a->getTime() == b->getTime()) );
    }

    bool compareVectors(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        yarp::sig::Vector * a = dynamic_cast<yarp::sig::Vector *>(ref);
        yarp::sig::Vector * b = dynamic_cast<yarp::sig::Vector *>(test);
        if(verbose)
        {
            std::cout << typeid(*ref).name() << '\n';
            std::cout << "A size : " << a->size() << "  B size: " << b->size() <<std::endl;
            for(auto i=0; i<a->size(); i++)
                std::cout << "a " << i << " a[i] " << (*a)[i] << " b[i] " << (*b)[i] << std::endl;
        }
        return *a == *b;
    }

    bool compareVectorsOfInt(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        yarp::sig::VectorOf<int> * a = dynamic_cast<yarp::sig::VectorOf<int> *>(ref);
        yarp::sig::VectorOf<int> * b = dynamic_cast<yarp::sig::VectorOf<int> *>(test);
        if(verbose)
        {
            std::cout << typeid(*ref).name() << '\n';
            std::cout << "A size : " << a->size() << "  B size: " << b->size() <<std::endl;
        }

        bool check = true;
        for(auto i=0; i<a->size(); i++)
        {
            if(verbose)    std::cout << "a " << i << " a[i] " << (*a)[i] << " b[i] " << (*b)[i] << std::endl;
            check &= (*a)[i] == (*b)[i];
        }
        return check;
    }

    bool compareMapGrid2D(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)     std::cout << typeid(*ref).name() << '\n';
        yarp::dev::MapGrid2D * a = dynamic_cast<yarp::dev::MapGrid2D *>(ref);
        yarp::dev::MapGrid2D * b = dynamic_cast<yarp::dev::MapGrid2D *>(test);

        size_t ax, ay, bx, by;
        a->getSize_in_cells(ax, ay);
        b->getSize_in_cells(bx, by);

        if( (ax==bx) && (ay == by) )
            si;
        altrimenti no;
    }

    bool compareValues(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        yarp::os::Value * a = dynamic_cast<yarp::os::Value *>(ref);
        yarp::os::Value * b = dynamic_cast<yarp::os::Value *>(test);
        return *a==*b;
    }

    bool compareMatrixes(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        yarp::sig::Matrix * a = dynamic_cast<yarp::sig::Matrix *>(ref);
        yarp::sig::Matrix * b = dynamic_cast<yarp::sig::Matrix *>(test);
        return *a==*b;
    }

    bool compareProperties(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        yarp::os::Property * a = dynamic_cast<yarp::os::Property *>(ref);
        yarp::os::Property * b = dynamic_cast<yarp::os::Property *>(test);

        if( a->check("5") && b->check("5"))
            if( a->find("5").asInt() && b->find("5").asInt() )
                si;
            altrimenti no;
        altrimenti no;
    }


    bool compareQuaternions(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        yarp::math::Quaternion * a = dynamic_cast<yarp::math::Quaternion *>(ref);
        yarp::math::Quaternion * b = dynamic_cast<yarp::math::Quaternion *>(test);

        return ( (a->x() == b->x()) && (a->y() == b->y()) );
    }

    bool compareImages(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        yarp::sig::ImageOf< yarp::sig::PixelRgb > * a = dynamic_cast<yarp::sig::ImageOf< yarp::sig::PixelRgb > *>(ref);
        yarp::sig::ImageOf< yarp::sig::PixelRgb > * b = dynamic_cast<yarp::sig::ImageOf< yarp::sig::PixelRgb > *>(test);


        uint8_t ar = (uint8_t) a->pixel(0,0).r;
        uint8_t ag = (uint8_t) a->pixel(0,0).g;
        uint8_t ab = (uint8_t) a->pixel(0,0).b;

        uint8_t br = (uint8_t) b->pixel(0,0).r;
        uint8_t bg = (uint8_t) b->pixel(0,0).g;
        uint8_t bb = (uint8_t) b->pixel(0,0).b;

        if(verbose)
        {
            std::cout << typeid(*ref).name() << '\n';
            std::cout << "A width " << a->width() << " A height " << a->height() << std::endl;
            std::cout << "B width " << b->width() << " B height " << b->height() << std::endl;

            printf("A r %d  B r %d\n", ar, br);
            printf("A g %d  B g %d\n", ag, bg);
            printf("A b %d  B b %d\n", ab, bb);
        }

        ar = (uint8_t) a->pixel(4,1).r;
        ag = (uint8_t) a->pixel(4,1).g;
        ab = (uint8_t) a->pixel(4,1).b;

        br = (uint8_t) b->pixel(4,1).r;
        bg = (uint8_t) b->pixel(4,1).g;
        bb = (uint8_t) b->pixel(4,1).b;

        if(verbose)
        {
            printf("A r %d  B r %d\n", ar, br);
            printf("A g %d  B g %d\n", ag, bg);
            printf("A b %d  B b %d\n", ab, bb);
        }

        if( //a->width() == b->width()            &&
            //a->height() == b->height()          &&
            ar == br  &&
            ag == bg  &&
            ab == bb)
                si;
        altrimenti no;


        if( a->width() == b->width()            &&
            a->height() == b->height()          &&
            a->pixel(0,0).r == b->pixel(0,0).r  &&
            a->pixel(0,0).g == b->pixel(0,0).g  &&
            a->pixel(0,0).b == b->pixel(0,0).b)
                si;
        altrimenti no;
    }

    bool compareVectors2D(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        yarp::math::Vec2D<double> * a = dynamic_cast<yarp::math::Vec2D<double> *>(ref);
        yarp::math::Vec2D<double> * b = dynamic_cast<yarp::math::Vec2D<double> *>(test);
        if(a->x == b->x  &&  a->y == b->y)
            si;
        altrimenti no;
    }

    // yarp::os::impl::PortCommand
    bool comparePortCommands(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        yarp::os::impl::PortCommand * a = dynamic_cast<yarp::os::impl::PortCommand *>(ref);
        yarp::os::impl::PortCommand * b = dynamic_cast<yarp::os::impl::PortCommand *>(test);

        if(verbose)
        {
            std::cout << typeid(*ref).name() << '\n';
            std::cout << "A ch " << a->ch << " A str " << a->str << std::endl;
            std::cout << "B ch " << b->ch << " B str " << b->str << std::endl;
        }
        if( (a->ch == b->ch) && (a->str == b->str) )
            si;
        altrimenti no;
    }

    // yarp::os::impl::PortCommand
    bool compareSounds(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        yarp::sig::Sound * a = dynamic_cast<yarp::sig::Sound *>(ref);
        yarp::sig::Sound * b = dynamic_cast<yarp::sig::Sound *>(test);
        if(a->get(2,0) == b->get(2,0) )
            si;
        altrimenti no;
    }

    bool compareBites(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        bool good = true;
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        yarp::os::ManagedBytes * a = dynamic_cast<yarp::os::ManagedBytes *>(ref);
        yarp::os::ManagedBytes * b = dynamic_cast<yarp::os::ManagedBytes *>(test);

        for(int i=0; i<10; i++)
        {
            good &= (a->bytes().get()[i] == b->bytes().get()[i]);
        }
        if(good) si; altrimenti no;
    }

    bool comparePiPi(yarp::os::Portable *ref, yarp::os::Portable *test)
    {
        if(verbose)  std::cout << typeid(*ref).name() << '\n';
        PiPi * a = dynamic_cast<PiPi *>(ref);
        PiPi * b = dynamic_cast<PiPi *>(test);
        if( ((*a).head == (*b).head) && (compareVectorsOfInt( &((*a).body), (&(*b).body))) )
            si;
        altrimenti no;
    }
};

#endif // PORTABLES_BEST_OF_H
