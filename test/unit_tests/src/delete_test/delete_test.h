#ifndef __DELETE_TEST_H
#define __DELETE_TEST_H

#include <QTest>
#include "../repositorycommon.h"

class DeleteTest : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void test_delete_entity();
    void test_delete_tree();
    void test_delete_sub_entity();
    void test_delete_sub_tree();

    void test_delete_entities_and_trees_mixed();
private:
    std::string fileSystemName_;
    std::shared_ptr<upns::Repository> repo_;
    std::shared_ptr<upns::Checkout> checkout_;

    void add_bunny(std::string path);
    void createTestdata();
};

#endif
