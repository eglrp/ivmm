#ifndef  JSON_H
#define  JSON_H


#include  <utility>
#include  <json/json.h>
namespace Json{
    class EasyPutValue:public Json::Value{
        public:
        EasyPutValue()=default;
        template<typename ...Args>
        EasyPutValue(Args&& ... args):Json::Value( ::std::forward<Args>(args)... ){}
        template<typename T>
            EasyPutValue& add(::std::string const& key, T&& value){
                (*this)[key] = ::std::forward<T>(value);
                return *this;
            }
        EasyPutValue& add(::std::string const& key, long const& value){
            (*this)[key] = static_cast<Int64>(value);
            return *this;
        }

        template<typename T>
        EasyPutValue& append(T&& value){
            Json::Value::append( std::forward<T>(value) );
            return *this;
        }
    };
}


#endif  /*JSON_H*/
