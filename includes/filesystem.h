#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <string>
#include <cstdlib>

const char* logl_root = "..";

class FileSystem
{
private:
  typedef std::string (*Builder) (const std::string& path);

public:
  static std::string getPath(const std::string& path)
  {
    static std::string(*pathBuilder)(std::string const &) = getPathBuilder();
    return (*pathBuilder)(path);
  }

private:
  static std::string const & getRoot()
  {
    //static char const * envRoot = getenv("LOGL_ROOT_PATH");
    //static char const * givenRoot = (envRoot != nullptr ? envRoot : logl_root);
    //static std::string root = (givenRoot != nullptr ? givenRoot : "");
    //return root;

      static std::string root = [] {
          char buffer[4096];
          size_t len = 0;

          if (getenv_s(&len, buffer, sizeof(buffer), "LOGL_ROOT_PATH") == 0 && len > 0)
              return std::string(buffer);

          return std::string(logl_root ? logl_root : "");
          }();

      return root;
  }

  //static std::string(*foo (std::string const &)) getPathBuilder()
  static Builder getPathBuilder()
  {
    if (getRoot() != "")
      return &FileSystem::getPathRelativeRoot;
    else
      return &FileSystem::getPathRelativeBinary;
  }

  static std::string getPathRelativeRoot(const std::string& path)
  {
    return getRoot() + std::string("/") + path;
  }

  static std::string getPathRelativeBinary(const std::string& path)
  {
    return "../" + path;
  }
};

// FILESYSTEM_H
#endif
