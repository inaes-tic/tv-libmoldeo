/*******************************************************************************

                                moFilterManager.h

  ****************************************************************************
  *                                                                          *
  *   This source is free software; you can redistribute it and/or modify    *
  *   it under the terms of the GNU General Public License as published by   *
  *   the Free Software Foundation; either version 2 of the License, or      *
  *  (at your option) any later version.                                    *
  *                                                                          *
  *   This code is distributed in the hope that it will be useful, but       *
  *   WITHOUT ANY WARRANTY; without even the implied warranty of             *
  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU      *
  *   General Public License for more details.                               *
  *                                                                          *
  *   A copy of the GNU General Public License is available on the World     *
  *   Wide Web at <http://www.gnu.org/copyleft/gpl.html>. You can also       *
  *   obtain it by writing to the Free Software Foundation,                  *
  *   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.         *
  *                                                                          *
  ****************************************************************************

  Copyright(C) 2006 Fabricio Costa

  Authors:
  Fabricio Costa

*******************************************************************************/

#ifndef __MOFILTERMANAGER_H
#define __MOFILTERMANAGER_H

#include "moTypes.h"
#include "moAbstract.h"
#include "moText.h"
#include "moVideoGraph.h"
#include "moResourceManager.h"
#include "moBuckets.h"
#include "moParam.h"
#include "moMathVector.h"

#define MO_TRACKER_TRACKED 0
#define MO_TRACKER_NOT_FOUND -1
#define MO_TRACKER_SMALL_DET -2
#define MO_TRACKER_MAX_ITERATIONS -3
#define MO_TRACKER_OOB -4
#define MO_TRACKER_LARGE_RESIDUE -5

class moTrackerFeature;

moDeclareExportedDynamicArray( moTrackerFeature*, moTrackerFeatureArray );

/// Característica de seguimiento ( sensado )
/**
*   Característica de seguimiento ( sensado )
*	Punto reconocido y sensado en un espacio bidimensional,
*	esta clase es abstracta y debe ser implementada en cada dispositivo de sensado y seguimiento
*	@see moTrackerSystemData
*	@see moTrackerFeatureArray
*	@see moFilterManager
*/
class LIBMOLDEO_API moTrackerFeature { //de GpuKLT_Feature

	public:

    bool                    is_object;              //!< once recognized
    bool                    is_parent;              //!< once recognized
    bool                    is_cursor;              //!< once recognized
    long                    stime;              //!< once recognized
    long                    sframe;              //!< once recognized
    long                    utime;              //!< once recognized
    long                    uframe;              //!< once recognized

    moTrackerFeatureArray   FeaturesCaptured;
    moTrackerFeature*       Parent;

    float					x,y;					 //!< Location
	float					normx, normy;            //!< Normalized Feature Coordinates [ 0 - 1 ]
	float					tr_x, tr_y;              //!< Feature position in the previous frame.
	float					v_x, v_y;                //!< Speed in the actual frame.
	float					vp_x, vp_y;              //!< Speed in the previous frame.
	float					a_x, a_y;                //!< Acceleration in the actual frame.
	float					ap_x, ap_y;                //!< Acceleration in the previous frame.
	float         t_x, t_y;                 //!< Torque in the actual frame.
	bool				    valid;					 //!< True for a valid feature point.
	int                     val;                     //!<Other states for valid feature point
	moVector2fArray         track;					 //!< list of feature positions in the past frames. Forms the feature tracks in video.

	//! Constructor
	moTrackerFeature();

	moTrackerFeature & operator = (const moTrackerFeature &src);

	//! Destructor
	virtual ~moTrackerFeature();

	//! Imprime los detalles de este punto
	void print();

	//! Imprime el valor de los valores trazados por este punto
	void printTracks();

	//! Update Feature Positions
	int updatePos(float kltConvergeThreshold, float kltSSDthresh, int kltborder, float delta, float res, float d1, float d2, float w, float h);
};

/// Matrix de características
/**
* Valores a ser tratados y manipulados históricamente.
* Memoria de sensado. Semántica incluida.
*
*/
class LIBMOLDEO_API moTrackerInstanceRecord {

  public:
    moTrackerInstanceRecord();
    virtual ~moTrackerInstanceRecord() {}
    moTrackerInstanceRecord & operator = (const moTrackerInstanceRecord &src);

    ///operador de copia, y seteo de timers
    long            m_Tick;


    /**
    *   Datos del baricentro.
    *   posición, velocidad, aceleración
    */
		moVector2f      m_Barycenter;
		moVector2f      m_BarycenterMotion;
		moVector2f      m_BarycenterAcceleration;

    /**
    *   Distribución espacial
    *   Variance: como se extienden los puntos en el área
    *   Speed Variance: como varian los puntos en el espacio
    *   Acceleration Variance: como se aceleran los puntos en el espacio
    *   Bounding Rectangle: el área más pequeña que incluye todos los puntos...
    */
		moVector2f      m_Variance;
		moVector2f      m_SpeedVariance;
		moVector2f      m_AccelerationVariance;
		moVector4f      m_BoundingRectangle;

		/**
		*   Velocidad promedio absoluta
		*   formula, la suma de las normas dividido la cantidad
		*/
    double          m_AbsoluteSpeedAverage;///cantidad de movimiento
    double          m_AbsoluteAccelerationAverage;///cantidad de cambios de velocidad
    double          m_AbsoluteTorqueAverage; ///cantidad de fuerzas
    double          m_BoundingRectangleAngle;
    int             m_BlobCandidates;///separa formas de fondo
		double          m_SurfaceCovered;///0 nada, 1 todo...

		int             m_ValidFeatures;
		int             m_DeltaValidFeatures;
		int             m_nFeatures;

		moVector2f      m_Max;
		moVector2f      m_Min;

};

moDeclareExportedDynamicArray( moTrackerInstanceRecord, moTrackerInstanceRecords );

#include "moTimeManager.h"

class LIBMOLDEO_API moTrackerSystemHistory : public moAbstract {
    public:
      moTrackerSystemHistory();
      virtual ~moTrackerSystemHistory();


      /// Empieza a grabar
      /**
      *   Prepara el arreglo para los registros, arranca el temporizador y habilita así
      *   poder grabar.
      *   El objeto padre tiene que llamar regularmente a Record cuando tenga datos
      *   y esta clase se ocupará de registrar en funcion de la granularidad...
      *   En un futuro se podría generar algun tipo de interpolacion, extrapolación para mejorar
      *   los valores de estos datos.
      *   @param maxtime tiempo máximo que durará la grabación  en milisegundos
      *   @param granularity intervalo entre cada grabación en milisegundos
      */
      void  StartRecording( long maxtime = 60000, long granularity = 30 );
      bool  IsRecording() {return m_Timer.Started(); }
      void  Record( moTrackerInstanceRecord& p_InstanceRecord, long p_Tick = -1 );
      ///Util cuando no hay señales interesantes...
      void  PauseRecording();
      void  ContinueRecording();
      void  StopRecording();
      void  Reset();
      long  CountRecords();
      bool  SaveToFile( moText filename );
      bool  SaveToXML( moText filename );
      moTrackerInstanceRecord& Get( int index );

    public:

      long        m_MaxTime;
      long        m_Granularity;
      long        m_nRecorded;
      double      m_RecordFactor;

      moTimer                     m_Timer;
      moTrackerInstanceRecords    m_History;
};

/// Sistema de características
/**
*	Clase abstracta, interfaz de acceso a conjunto de datos de seguimiento [tracking]
*	esta clase es abstracta y debe ser implementada en cada dispositivo de seguimiento
*	@see moTrackerFeature
*	@see moTrackerFeatureArray
*	@see moFilterManager
*/
class LIBMOLDEO_API moTrackerSystemData {
	public:

    moTrackerSystemData();
    moTrackerSystemData( int ZoneW, int ZoneH );
    moTrackerSystemData( int ZoneW, int ZoneH, int ZoneCW, int ZoneCH );
    virtual ~moTrackerSystemData();

		/**
		*   Devuelve las características máximas
		*/
		virtual int GetFeaturesCount();

		/**
		*   Devuelve las características válidas. Son aquellas reconocidas de un cuadro a otro.
		*/
    virtual int GetValidFeatures();
    virtual int GetDeltaValidFeatures();

		/**
		*   Devuelve el vector de posición del baricentro.
		*/
		virtual moVector2f GetBarycenter();

		/**
		*   Devuelve el vector de velocidad del baricentro.
		*/
		virtual moVector2f GetBarycenterMotion();

		/**
		*   Devuelve el vector de aceleración del baricentro.
		*/
		virtual moVector2f GetBarycenterAcceleration();

		/**
		*   Devuelve la varianza de las posiciones de las características.
		*/
		virtual moVector2f GetVariance();

		/**
		*   Devuelve la varianza de la velocidad de las características
		*/
		virtual moVector2f GetSpeedVariance();

    /**
		*   Devuelve la varianza de la aceleración de las características
		*/
		virtual moVector2f GetAccelerationVariance();

    /**
		*   Devuelve el rectángulo más pequeño que rodea las características.
		*/
		virtual moVector4f GetBoundingRectangle();

		virtual moVector2f GetMax();
		virtual moVector2f GetMin();
		virtual void SetMax( float x, float y );
		virtual void SetMin( float x, float y );


		virtual moTrackerFeature* GetFeature(int i);
		virtual moTrackerFeatureArray& GetFeatures();

		virtual moVideoFormat& GetVideoFormat() { return m_VideoFormat; }

		virtual void SetBarycenter( float b_x, float b_y) { m_ActualRecord.m_Barycenter = moVector2f(b_x,b_y); }
		virtual void SetBarycenterMotion( float b_x, float b_y) { m_ActualRecord.m_BarycenterMotion = moVector2f(b_x,b_y); }
		virtual void SetBarycenterAcceleration( float b_x, float b_y) { m_ActualRecord.m_BarycenterAcceleration = moVector2f(b_x,b_y); }

		virtual void SetVariance( float v_x, float v_y) { m_ActualRecord.m_Variance = moVector2f(v_x,v_y); }
		virtual void SetSpeedVariance( float v_x, float v_y) { m_ActualRecord.m_SpeedVariance = moVector2f(v_x,v_y); }
		virtual void SetAccelerationVariance( float v_x, float v_y) { m_ActualRecord.m_AccelerationVariance = moVector2f(v_x,v_y); }
		virtual void SetBoundingRectangle( float r_x, float r_y, float s_x, float s_y ) { m_ActualRecord.m_BoundingRectangle = moVector4f(r_x,r_y,s_x,s_y); }

		virtual void SetValidFeatures( int validfeatures) { m_ActualRecord.m_ValidFeatures = validfeatures; }
		virtual void SetDeltaValidFeatures( int deltavalidfeatures) { m_ActualRecord.m_DeltaValidFeatures = deltavalidfeatures; }

		virtual void SetAbsoluteSpeedAverage( float speedaverage ) { m_ActualRecord.m_AbsoluteSpeedAverage = speedaverage; }
		virtual void SetAbsoluteAccelerationAverage( float accaverage ) { m_ActualRecord.m_AbsoluteAccelerationAverage = accaverage; }
		virtual void SetAbsoluteTorqueAverage( float toraverage ) { m_ActualRecord.m_AbsoluteTorqueAverage = toraverage; }


    ///Transforma el vector de posicion (0..1) al indice de zona de la matriz cuadrada
    virtual int PositionToZone( float x, float y );

    ///Transforma el indice de zona al vector posicion
    virtual moVector2f ZoneToPosition( int zone );


    ///Transforma el vector de posicion (0..1) a coordenadas polares luego al indice de zona de la matriz
    virtual int PositionToZoneC( float x, float y );

    ///Transforma el indice de zona al vector posicion circular!
    virtual moVector2f ZoneToPositionC( int zone );

    virtual void SetPositionMatrix( float x, float y, int nfeatures );
    virtual void SetPositionMatrix( moVector2f pos, int nfeatures );
    virtual int GetPositionMatrix( float x, float y );
    virtual int GetPositionMatrix( moVector2f pos );

    virtual void SetMotionMatrix( float x, float y, int nfeatures );
    virtual void SetMotionMatrix( moVector2f pos,  int nfeatures );
    virtual int GetMotionMatrix( float x, float y );
    virtual int GetMotionMatrix( moVector2f pos );

    virtual void SetAccelerationMatrix( float x, float y, int nfeatures );
    virtual void SetAccelerationMatrix( moVector2f pos,  int nfeatures );
    virtual int GetAccelerationMatrix( float x, float y );
    virtual int GetAccelerationMatrix( moVector2f pos );

    virtual void SetPositionMatrixC( float x, float y, int nfeatures );
    virtual int GetPositionMatrixC( float x, float y );
    virtual int GetPositionMatrixC( moVector2f pos );
    virtual int GetPositionMatrixC( int zone );

    virtual void SetMotionMatrixC( float x, float y, int nfeatures );
    virtual int GetMotionMatrixC( float x, float y );
    virtual int GetMotionMatrixC( moVector2f pos );
    virtual int GetMotionMatrixC( int zone );

    virtual void ResetMatrix();
    virtual void DrawFeatures( int w, int h, float offsetx, float offsety );

    virtual void SetMaxFeatures( int p_nFeatures ) { m_ActualRecord.m_nFeatures = p_nFeatures; }
    virtual int GetMaxFeatures() { return m_ActualRecord.m_nFeatures; }

    void        Record();
    moTrackerSystemHistory&   GetHistory() { return m_History; }

    int             m_ZoneW;
    int             m_ZoneH;
    int             m_ZoneCW;
    int             m_ZoneCH;
    int             m_Zones;
    int             m_ZonesC;
    float**         m_Distancias;
    int**           m_Pares;
    int             nPares;

	protected:

    float           _zonewf;
    float           _zonehf;
    float           _zonecwf;
    float           _zonechf;

    moVideoFormat	m_VideoFormat;

    moTrackerFeatureArray m_Features;

    moTrackerInstanceRecord m_ActualRecord;

    moTrackerSystemHistory  m_History;
    /**
    *   Matrices de activación dinámicas.
    *   Circulares cemtradas en el baricentro, y cuadradas cubriendo toda el área.
    */

		int             *m_PositionMatrix; ///4 x 4 zones
		int             *m_MotionMatrix; ///4 x 4 zones
		int             *m_AccelerationMatrix; ///4 x 4 zones

		int             *m_CircularPositionMatrix; ///12 semitones, 3 levels
		int             *m_CircularMotionMatrix; ///12 semitones, 3 levels

};


/**
*	Administrador de recursos de procesamiento o filtro de diferentes datos
*	este administrador de filtros permite procesar disintas fuentes de datos
*	como son el seguimiento de características de una imagen en movimiento  o bien
*	la transformación de un tipo de dato a otro
*	@see moTrackerData
*	@see moTrackerFeature
*	@see moTrackerFeatureArray
*	@see moFilterManager
*/
class LIBMOLDEO_API moFilterManager : public moResource
{
	public:

		moFilterManager();
		virtual ~moFilterManager();

		virtual MOboolean Init();
		virtual MOboolean Finish();

	protected:


};


#endif
