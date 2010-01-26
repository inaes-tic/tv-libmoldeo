/*******************************************************************************

                              moPostPlugin.cpp

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
  Andr� Colubri

*******************************************************************************/

#include "moPostPlugin.h"

#include "moArray.cpp"
moDefineDynamicArray( moPostPluginsArray )

moPostEffectFactory::~moPostEffectFactory() {
}


//===========================================
//
//                                   moPostPlugin
//
//===========================================

void moPostPlugin::Load(moText plugin_file)
{
    name = plugin_file;
    handle = moLoadPlugin(plugin_file);

    if(!handle) {
	#if !defined(WIN32)
        cerr << "Cannot open library: " << dlerror() << '\n';
	#else
		CHAR szBuf[80];
		DWORD dw = GetLastError();
		sprintf(szBuf, "%s failed: GetLastError returned %u\n",
			(char*)plugin_file, dw);
		//MessageBox(NULL, szBuf, "Error", MB_OK);

		cerr << "Cannot open library: " << szBuf <<'\n';
	#endif
    }

    #if defined(_WIN32)
	FARPROC farp;
	farp = GetProcAddress(handle, "DestroyPostEffectFactory");
    CreatePostEffectFactory = CreatePostEffectFactoryFunction(GetProcAddress(handle, "CreatePostEffectFactory"));
	DestroyPostEffectFactory = DestroyPostEffectFactoryFunction(GetProcAddress(handle, "DestroyPostEffectFactory"));
    #else
    CreatePostEffectFactory = CreatePostEffectFactoryFunction(dlsym(handle, "CreatePostEffectFactory"));
	DestroyPostEffectFactory = DestroyPostEffectFactoryFunction(dlsym(handle, "DestroyPostEffectFactory"));
    #endif

	if(this->CreatePostEffectFactory!=NULL)
		m_factory = this->CreatePostEffectFactory();

}

void moPostPlugin::Unload()
{
	this->DestroyPostEffectFactory();

    CreatePostEffectFactory = NULL;
	DestroyPostEffectFactory = NULL;

    moUnloadPlugin(handle);
}


moPostEffect* moPostPlugin::Create() {

	moPostEffect **narray;

	if(m_factory!=NULL) {
		moPostEffect *pfx = m_factory->Create();

		if(pfx==NULL) return NULL;

		if(n==0) {//primera vez
			array = new moPostEffect* [1];
		} else if(n>0) {//array dinamico
			narray = new moPostEffect* [n+1];//generamos el nuevo vacio
			for(int i=0;i<n;i++) narray[i] = array[i];//copiamos el array
			delete [] array;
			array = narray;
		}

		array[n] = pfx;
		n++;
		return pfx;
	}
	return NULL;
}

void moPostPlugin::Destroy(moPostEffect *PostEfecto) {

	moPostEffect **narray;
	int i,j;

	if(m_factory!=NULL) {

		for(j=0;j<n;j++)
			if(array[j]==PostEfecto) break;

		m_factory->Destroy(PostEfecto);

		if(n==1) {//muere
			delete [] array;
		} else if(n>1) {//array dinamico
			narray = new moPostEffect* [n-1];//generamos el nuevo vacio
			for(i=0;i<j;i++) narray[i] = array[i];//copiamos el array hasta el j(destruido)
			for(i=j;i<(n-1);i++) narray[i] = array[i+1];//copiamos el array desde el j
			delete [] array;
			array = narray;
		}
		n--;
		return;
	}
	return;
}

//===========================================
//
//                             moPostPluginsArray
//
//===========================================
/*
void moPostPluginsArray::Add(moPostPlugin* plugin)
{
    if(length < max_length)
    {
        if(plugin!=NULL) {
			cout << "plugin agregado\n";
			array[length] = plugin;
			length++;
		} else {
			cout << "error: plugin no fue creado!\n";
		}
    }
}

void moPostPluginsArray::Init(MOuint nplugins)
{
    max_length = 256;
    length = nplugins;
    array = new moPostPlugin*[max_length];
    for(MOuint i = 0; i < max_length; i++) array[i] = NULL;
}

void moPostPluginsArray::Finish()
{
    if(array != NULL)
    {
        delete[] array;
        array = NULL;
    }

    length = 0;
}

*/
LIBMOLDEO_API moPostEffect* moNewPostEffect(moText effect_name, moPostPluginsArray &plugins)
{
    // Creando el nombre complete del plugin(incluyendo ruta por defecto)
    // a partir del nombre del efecto.
    moText complete_name;

    if(!stricmp(effect_name, "nil")) return NULL;

    #if defined(_WIN32)
    complete_name = "plugins/posteffects/" + effect_name;
		#ifdef _DEBUG
		complete_name+= "d";
		#endif
    complete_name += ".dll";
    #else
    complete_name = "plugins/posteffects/lib" + effect_name;
		#ifdef _DEBUG
		complete_name+= "d";
		#endif
    complete_name += ".so";
    #endif
		//printf("completename:%s\n",complete_name);

    // Indice del plugin que se utilizara para crear a este efecto.
    int plg_index = -1;

    // First, revisa que el plugin ya no haya sido cargado antes.
    for(MOuint i = 0; i < plugins.Count(); i++)
        if(!stricmp(plugins[i]->GetName(), complete_name))
        {
            plg_index = i;
            break;
        }

    if(plg_index == -1)
    {
        // Es la primera vez que se intenta cargar este plugin. Se lo agrega al array.
        // Falta control de errores(que pasa si la libreria no carga, etc?) :-)
        plg_index = plugins.Count();
		moPostPlugin *pplugin = new moPostPlugin(complete_name);
        plugins.Add( pplugin );
    }

    // El plugin crea al efecto!
	if(plugins[plg_index]->m_factory!=NULL)
	return plugins[plg_index]->Create();
	else return NULL;
}


LIBMOLDEO_API void moDeletePostEffect(moPostEffect *posteffect, moPostPluginsArray &plugins)
{
    // Creando el nombre complete del plugin(incluyendo ruta por defecto)
    // a partir del nombre del efecto.
    moText complete_name;

    if(!stricmp(posteffect->GetName(), "")) return;

    #if defined(_WIN32)
    complete_name = moText("plugins/posteffects/") + moText(posteffect->GetName());
		#ifdef _DEBUG
		complete_name+= "d";
		#endif
    complete_name += ".dll";
    #else
    complete_name = moText("plugins/posteffects/lib") + moText(posteffect->GetName());
		#ifdef _DEBUG
		complete_name+= "d";
		#endif
    complete_name += ".so";
    #endif

    // Indice del plugin que se utilizara para crear a este efecto.
    int plg_index = -1;

    // First, revisa que el plugin ya no haya sido cargado antes.
    for(MOuint i = 0; i < plugins.Count(); i++)
        if(!stricmp(plugins[i]->GetName(), complete_name))
        {
            plg_index = i;
            break;
        }

    if(plg_index == -1)
    {
        return;
    }

    plugins[plg_index]->Destroy(posteffect);
}