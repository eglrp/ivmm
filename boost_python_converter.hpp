#include  <boost/python.hpp>
#include  <boost/python/stl_iterator.hpp>
#include  <vector>
#include  <list>
#include  <map>
#include  <unordered_map>





namespace __py = boost::python;


template<typename T>
T& handle(T& t){
    return t;
}

template<typename T>
T handle(const T& t){
    return t;
}

__py::object handle(PyObject* obj){
    __py::handle<> handle(obj);
    return __py::object(handle);
}


template<typename T>
struct try_convert_to_python{
    static T convert(T const& t){
        return t;
    }
};

template<typename T>
struct try_convert_to_python<std::vector<T> >{
    static PyObject* convert(const std::vector<T>& vec){
        __py::list list;
        for(auto& t : vec){
            list.append(handle(try_convert_to_python<T>::convert(t)));
        }
        return __py::incref(list.ptr());
    }
};

template<typename T>
struct try_convert_to_python<std::list<T> > {
    static PyObject* convert(const std::list<T>& vec){
        __py::list list;
        for(auto& t:vec){
            list.append(handle(try_convert_to_python<T>::convert(t)));
        }
        return __py::incref(list.ptr());
    }
};

template<typename Key, typename Val>
struct try_convert_to_python<std::map<Key, Val> >{
    static PyObject* convert(const std::map<Key, Val>& mp){
        __py::dict dict;
        for(auto& pair : mp){
            dict[handle(try_convert_to_python<Key>::convert(pair.first))] = handle(try_convert_to_python<Val>::convert(pair.second));
        }
        return __py::incref(dict.ptr());
    }
};

template<typename Key, typename Val>
struct try_convert_to_python<std::pair<Key, Val> >{
    static PyObject* convert(const std::pair<Key,Val>& pair){
        __py::tuple tup = __py::make_tuple(handle(try_convert_to_python<Key>::convert(pair.first)), handle(try_convert_to_python<Val>::convert(pair.second)));
        return __py::incref(tup.ptr());
    }
};

template<typename Key, typename Val>
struct try_convert_to_python<std::unordered_map<Key, Val> >{
    static PyObject* convert(const std::unordered_map<Key, Val>& mp){
        __py::dict dict;
        for(auto& pair : mp){
            dict[handle(try_convert_to_python<Key>::convert(pair.first))] = handle(try_convert_to_python<Val>::convert(pair.second));
        }
        return __py::incref(mp.ptr);
    }
};

std::map<int, std::vector<int> > test(){
    std::map<int, std::vector<int> > m{{1,{1,2,3}}, {2,{2,3,4}}};
    return m;
}

struct to_python_converter{
    template<typename T>
        to_python_converter& to_python(){
            __py::to_python_converter<T, try_convert_to_python<T> >();
            return *this;
        }
};

/// @brief Type that allows for registration of conversions from
///        python iterable types.
struct from_iterable_converter{
    /// @note Registers converter from a python interable type to the
    ///       provided type.
    template <typename Container>
        from_iterable_converter&
        from_python(){
            boost::python::converter::registry::push_back(
                    &from_iterable_converter::convertible,
                    &from_iterable_converter::construct<Container>,
                    boost::python::type_id<Container>());
            return *this;
        }

    /// @brief Check if PyObject is iterable.
    static void* convertible(PyObject* object){
        return PyObject_GetIter(object) ? object : NULL;
    }

    /// @brief Convert iterable PyObject to C++ container type.
    ///
    /// Container Concept requirements:
    ///
    ///   * Container::value_type is CopyConstructable.
    ///   * Container can be constructed and populated with two iterators.
    ///     I.e. Container(begin, end)
    template <typename Container>
        static void construct(
                PyObject* object,
                boost::python::converter::rvalue_from_python_stage1_data* data){
            namespace python = boost::python;
            // Object is a borrowed reference, so create a handle indicting it is
            // borrowed for proper reference counting.
            python::handle<> handle(python::borrowed(object));

            // Obtain a handle to the memory block that the converter has allocated
            // for the C++ type.
            typedef python::converter::rvalue_from_python_storage<Container>
                storage_type;
            void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

            typedef python::stl_input_iterator<typename Container::value_type>
                iterator;

            // Allocate the C++ type into the converter's memory block, and assign
            // its handle to the converter's convertible variable.  The C++
            // container is populated by passing the begin and end iterators of
            // the python object to the container's constructor.
            data->convertible = new (storage) Container(
                    iterator(python::object(handle)), // begin
                    iterator());                      // end
        }
};
